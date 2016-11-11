/*
Modified from https://forums.adafruit.com/viewtopic.php?f=19&t=89049
 */

package bmp280

import (
	"log"
	"time"
	"errors"
	"github.com/kidoman/embd"
	"fmt"
	"math"
)

const (
	BMP_ADDRESS = 0x77 // BMP280 I2C Address

	// BMP280 registers
	BMP280_REGISTER_DIG_T1 = 0x88
	BMP280_REGISTER_DIG_T2 = 0x8A
	BMP280_REGISTER_DIG_T3 = 0x8C
	BMP280_REGISTER_DIG_P1 = 0x8E
	BMP280_REGISTER_DIG_P2 = 0x90
	BMP280_REGISTER_DIG_P3 = 0x92
	BMP280_REGISTER_DIG_P4 = 0x94
	BMP280_REGISTER_DIG_P5 = 0x96
	BMP280_REGISTER_DIG_P6 = 0x98
	BMP280_REGISTER_DIG_P7 = 0x9A
	BMP280_REGISTER_DIG_P8 = 0x9C
	BMP280_REGISTER_DIG_P9 = 0x9E
	BMP280_REGISTER_CHIPID = 0xD0
	BMP280_REGISTER_VERSION = 0xD1
	BMP280_REGISTER_SOFTRESET = 0xE0
	BMP280_REGISTER_CONTROL = 0xF4
	BMP280_REGISTER_CONFIG  = 0xF5
	BMP280_REGISTER_STATUS = 0xF3
	BMP280_REGISTER_TEMPDATA_MSB = 0xFA
	BMP280_REGISTER_TEMPDATA_LSB = 0xFB
	BMP280_REGISTER_TEMPDATA_XLSB = 0xFC
	BMP280_REGISTER_PRESSDATA_MSB = 0xF7
	BMP280_REGISTER_PRESSDATA_LSB = 0xF8
	BMP280_REGISTER_PRESSDATA_XLSB = 0xF9

	QNH=1020 // Reference pressure in hPa

	BUFSIZE=256 // Buffer size for reading data from BMP
)

type BMPData struct {
	Temperature     float64
	Pressure	float64
	Altitude	float64
	T		time.Time
}

type BMP280 struct {
	i2cbus              embd.I2CBus
	PowerMode           byte
	Freq                byte
	Filter              byte
	TempRes             byte
	PresRes             byte

	config              byte
	control             byte

	digT1, digT2, digT3 float64
	digP1, digP2, digP3 float64
	digP4, digP5, digP6 float64
	digP7, digP8, digP9 float64

	C                   <-chan *BMPData
	CBuf                <-chan *BMPData
	cClose              chan bool
}

/*
NewBMP280 returns a BMP280 object with the chosen settings
powerMode = 0 (sleep), 1 (forced), 2 (forced), [3] (normal)
freq = 0 (0.5ms), 1 (62.5ms), 2 (125ms), 3 (250ms), [4] (500ms), 5 (1000ms), 6 (2000ms), 7 (4000ms)
filter = 0, 1, 2, 3, [4], 5, 6, 7
tempRes = 0 (no temp), 1 (16 bit), 2 (17 bit), 3 (18 bit), 4 (19 bit), 5 (20 bit)
presRes = 0 (no pres), 1 (16 bit), 2 (17 bit), 3 (18 bit), 4 (19 bit), 5 (20 bit)
*/
func NewBMP280(powerMode, freq, filter, tempRes, presRes byte) (bmp *BMP280, err error) {
	bmp.PowerMode = powerMode
	bmp.Freq = freq
	bmp.Filter = filter
	bmp.TempRes = tempRes
	bmp.PresRes = presRes

	bmp.config = (bmp.Freq << 5) + (filter << 2) // combine bits for config
	bmp.control = (tempRes << 5) + (presRes << 2) + powerMode // combine bits for control

	log.Println("CONFIG: ", bmp.config)
	log.Println("CTRL_MEAS: ", bmp.control)

	bmp.i2cbus = embd.NewI2CBus(2)

	var (
		control byte
		config  byte
	)
	// check sensor id 0x58=BMP280
	if (bmp.i2cRead(BMP280_REGISTER_CHIPID) != 0x58) {
		err = errors.New("BMP280: Wrong ChipID")
	}
	bmp.i2cWrite(BMP280_REGISTER_SOFTRESET, 0xB6) // reset sensor
	time.Sleep(0.2)
	bmp.i2cWrite(BMP280_REGISTER_CONTROL, bmp.control) //
	time.Sleep(0.2)
	bmp.i2cWrite(BMP280_REGISTER_CONFIG, bmp.config)  //
	time.Sleep(0.2)
	control, err = bmp.i2cRead(BMP280_REGISTER_CONTROL) // check the control register again
	if err != nil {
		err = fmt.Errorf("BMP280 Error: Couldn't check control register: %v", err)
		return
	}
	config, err = bmp.i2cRead(BMP280_REGISTER_CONFIG)// check the config register
	if err != nil {
		err = fmt.Errorf("BMP280 Error: Couldn't check control register: %v", err)
		return
	}
	log.Println("config: ", config)
	log.Println("control: ", control)

	bmp.ReadCorrectionSettings()
	go bmp.readSensor()
	time.Sleep(50 * time.Millisecond)
	return
}

// CloseBMP closes the BMP280
func (bmp *BMP280) CloseBMP() {
	// Nothing to do bitwise
	bmp.cClose<- true
}

// Frequency returns the sampling frequency, in Hz
func (bmp *BMP280) Frequency() float64 {
	// freq = 0 (0.5ms), 1 (62.5ms), 2 (125ms), 3 (250ms), [4] (500ms), 5 (1000ms), 6 (2000ms), 7 (4000ms)
	switch bmp.Freq {
	case 0:
		return 200
	case 1:
		return 16
	case 2:
		return 8
	case 3:
		return 4
	case 4:
		return 2
	case 5:
		return 1
	case 6:
		return 0.5
	default:
		return 0.25
	}
}

func (bmp *BMP280) ReadCorrectionSettings() error {
	uRegMap := map[*uint16]byte{
		&bmp.digT1: BMP280_REGISTER_DIG_T1, &bmp.digP1: BMP280_REGISTER_DIG_P1,
	}

	regMap := map[*int16]byte{
		&bmp.digT2: BMP280_REGISTER_DIG_T2, &bmp.digT3: BMP280_REGISTER_DIG_T3,
		&bmp.digP2: BMP280_REGISTER_DIG_P2, &bmp.digP3: BMP280_REGISTER_DIG_P3,
		&bmp.digP4: BMP280_REGISTER_DIG_P4, &bmp.digP5: BMP280_REGISTER_DIG_P5,
		&bmp.digP6: BMP280_REGISTER_DIG_P6, &bmp.digP7: BMP280_REGISTER_DIG_P7,
		&bmp.digP8: BMP280_REGISTER_DIG_P8, &bmp.digP9: BMP280_REGISTER_DIG_P9,
	}

	for p, reg := range uRegMap {
		v, err := bmp.i2cRead2u(reg)
		if err != nil {
			log.Printf("BMP280 Warning: error reading sensor data from %x", reg)
			return err
		}
		*p = float64(v)
	}

	for p, reg := range regMap {
		v, err := bmp.i2cRead2(reg)
		if err != nil {
			log.Printf("BMP280 Warning: error reading sensor data from %x", reg)
			return err
		}
		*p = float64(v)
	}

	log.Printf("digT1: %f, digT2: %f, digT3: %f\n", bmp.digT1, bmp.digT2, bmp.digT3)
	log.Printf("digP1: %f, digP2: %f, digP3: %f\n", bmp.digP1, bmp.digP2, bmp.digP3)
	log.Printf("digP4: %f, digP5: %f, digP6: %f\n", bmp.digP4, bmp.digP5, bmp.digP6)
	log.Printf("digP7: %f, digP8: %f, digP9: %f\n", bmp.digP7, bmp.digP8, bmp.digP9)

	return
}

func (bmp *BMP280) readSensor() {
	var (
		raw_temp_msb           int8
		raw_temp_lsb           int8
		raw_temp_xlsb          int8
		raw_temp               int16

		raw_press_msb          int8
		raw_press_lsb          int8
		raw_press_xlsb         int8
		raw_press              int16

		temp, press, altitude  float64
		err                    error

		t                  time.Time
	)

	regMap := map[*int16]byte{
		&raw_temp_msb: BMP280_REGISTER_TEMPDATA_MSB, &raw_temp_lsb: BMP280_REGISTER_TEMPDATA_LSB,
		&raw_temp_xlsb: BMP280_REGISTER_TEMPDATA_XLSB, &raw_press_msb: BMP280_REGISTER_PRESSDATA_MSB,
		&raw_temp_lsb: BMP280_REGISTER_PRESSDATA_LSB, &raw_temp_xlsb: BMP280_REGISTER_PRESSDATA_XLSB,
	}

	cC := make(chan *BMPData)
	defer close(cC)
	bmp.C = cC
	cBuf := make(chan *BMPData, BUFSIZE)
	defer close(cBuf)
	bmp.CBuf = cBuf
	bmp.cClose = make(chan bool)
	defer close(bmp.cClose)

	clock := time.NewTicker(time.Duration(int(1000/bmp.Frequency()+0.5)) * time.Millisecond)
	//TODO westphae: use the clock to record actual time instead of a timer
	defer clock.Stop()

	makeBMPData := func() *BMPData {
		d := BMPData{
			Temperature: temp,
			Pressure: press,
			Altitude: altitude,
			T: t,
		}
		return &d
	}

	for {
		select {
		case t = <-clock.C: // Read sensor data:
			for p, reg := range regMap {
				*p, err = bmp.i2cRead(reg)
				if err != nil {
					log.Printf("BMP280 Warning: error reading sensor data from %x", reg)
					continue
				}
			}

			raw_temp = (raw_temp_msb << 12) + (raw_temp_lsb << 4) + (raw_temp_xlsb >> 4) // combine 3 bytes  msb 12 bits left, lsb 4 bits left, xlsb 4 bits right
			log.Printf("BMP280:  raw_temp_msb: %d,  raw_temp_lsb: %d,  raw_temp_xlsb: %d\n",raw_temp_msb, raw_temp_xlsb, raw_temp_xlsb)
			log.Printf("BMP280:  raw_temp: %d\n", raw_temp)

			raw_press = (raw_press_msb << 12) + (raw_press_lsb << 4) + (raw_press_xlsb >> 4) // combine 3 bytes  msb 12 bits left, lsb 4 bits left, xlsb 4 bits right
			log.Printf("BMP280: raw_press_msb: %d, raw_press_lsb: %d, raw_press_xlsb: %d\n",raw_press_msb, raw_press_xlsb, raw_press_xlsb)
			log.Printf("BMP280: raw_press: %d\n", raw_press)

			temp, press = bmp.calcTempPress(float64(raw_temp), float64(raw_press))
			altitude = bmp.calcAltitude(temp, press)
		case cC<- makeBMPData(): // Send the latest values
		case cBuf<- makeBMPData():
		case <-bmp.cClose: // Stop the goroutine, ease up on the CPU
			break
		}
	}
}

func (bmp *BMP280) calcTempPress(raw_temp, raw_press float64) (temp, press float64) {
	var (
		var1, var2 float64
		t_fine     float64
	)

	var1 = (raw_temp / 16384.0 - bmp.digT1 / 1024.0) * bmp.digT2 // formula for temperature from datasheet
	var2 = (raw_temp / 131072.0 - bmp.digT1 / 8192.0) * (raw_temp / 131072.0 - bmp.digT1 / 8192.0) * bmp.digT3 // formula for temperature from datasheet
	t_fine = (var1 + var2) // need for pressure calculation
	temp = t_fine / 5120.0 // formula for temperature from datasheet

	var1 = t_fine / 2.0 - 64000.0 // formula for pressure from datasheet
	var2 = var1 * var1 * bmp.digP6 / 32768.0 // formula for pressure from datasheet
	var2 = var2 + var1 * bmp.digP5 * 2 // formula for pressure from datasheet
	var2 = var2 / 4.0 + bmp.digP4 * 65536.0 // formula for pressure from datasheet
	var1 = (bmp.digP3 * var1 * var1 / 524288.0 + bmp.digP2 * var1) / 524288.0 // formula for pressure from datasheet
	var1 = (1.0 + var1 / 32768.0) * bmp.digP1 // formula for pressure from datasheet
	press = 1048576.0 - raw_press // formula for pressure from datasheet
	press = (press - var2 / 4096.0) * 6250.0 / var1 // formula for pressure from datasheet
	var1 = bmp.digP9 * press * press / 2147483648.0 // formula for pressure from datasheet
	var2 = press * bmp.digP8 / 32768.0 // formula for pressure from datasheet
	press = press + (var1 + var2 + bmp.digP7) / 16.0 // formula for pressure from datasheet

	return
}

func (bmp *BMP280) calcAltitude(temp, press float64) (altitude float64) {
	altitude = 44330.0 * (1.0 - math.Pow(press / (QNH * 100), (1.0 / 5.255))) // formula for altitude from airpressure
	log.Printf("BMP280: temperature: %3.2f C, pressure: %4.1f hPa, altitude: %5.0f m\n", temp, press/10, altitude)

	return
}

func (bmp *BMP280) i2cWrite(register, value byte) (err error) {
	if errWrite := bmp.i2cbus.WriteByteToReg(BMP_ADDRESS, register, value); errWrite != nil {
		err = fmt.Errorf("BMP280 Error writing %X to %X: %s\n",
			value, register, errWrite.Error())
	} else {
		time.Sleep(time.Millisecond)
	}
	return
}

func (bmp *BMP280) i2cRead(register byte) (value uint8, err error) {
	value, errWrite := bmp.i2cbus.ReadByteFromReg(BMP_ADDRESS, register)
	if errWrite != nil {
		err = fmt.Errorf("BMP280 error: %s", errWrite.Error())
	}
	return
}

func (bmp *BMP280) i2cRead2(register byte) (value int16, err error) {
	v, errWrite := bmp.i2cbus.ReadWordFromReg(BMP_ADDRESS, register)
	if errWrite != nil {
		err = fmt.Errorf("BMP280 Error reading %x: %s\n", register, err.Error())
	} else {
		value = int16(v)
	}
	return
}
func (bmp *BMP280) i2cRead2u(register byte) (value uint16, err error) {
	v, errWrite := bmp.i2cbus.ReadWordFromReg(BMP_ADDRESS, register)
	if errWrite != nil {
		err = fmt.Errorf("BMP280 Error reading %x: %s\n", register, err.Error())
	} else {
		value = uint16(v)
	}
	return
}
