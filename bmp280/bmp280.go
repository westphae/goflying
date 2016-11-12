/*
Modified from https://forums.adafruit.com/viewtopic.php?f=19&t=89049
*/

package bmp280

import (
	"fmt"
	"github.com/kidoman/embd"
	_ "github.com/kidoman/embd/host/all"
	_ "github.com/kidoman/embd/host/rpi"
	"log"
	"math"
	"time"
)

const (
	BMP_ADDRESS = 0x76 // BMP280 I2C Address (or 0x77)

	// BMP280 registers
	BMP280_REGISTER_DIG_T1         = 0x88
	BMP280_REGISTER_DIG_T2         = 0x8A
	BMP280_REGISTER_DIG_T3         = 0x8C
	BMP280_REGISTER_DIG_P1         = 0x8E
	BMP280_REGISTER_DIG_P2         = 0x90
	BMP280_REGISTER_DIG_P3         = 0x92
	BMP280_REGISTER_DIG_P4         = 0x94
	BMP280_REGISTER_DIG_P5         = 0x96
	BMP280_REGISTER_DIG_P6         = 0x98
	BMP280_REGISTER_DIG_P7         = 0x9A
	BMP280_REGISTER_DIG_P8         = 0x9C
	BMP280_REGISTER_DIG_P9         = 0x9E
	BMP280_REGISTER_CHIPID         = 0xD0
	BMP280_REGISTER_VERSION        = 0xD1
	BMP280_REGISTER_SOFTRESET      = 0xE0
	BMP280_REGISTER_CONTROL        = 0xF4
	BMP280_REGISTER_CONFIG         = 0xF5
	BMP280_REGISTER_STATUS         = 0xF3
	BMP280_REGISTER_TEMPDATA_MSB   = 0xFA
	BMP280_REGISTER_TEMPDATA_LSB   = 0xFB
	BMP280_REGISTER_TEMPDATA_XLSB  = 0xFC
	BMP280_REGISTER_PRESSDATA_MSB  = 0xF7
	BMP280_REGISTER_PRESSDATA_LSB  = 0xF8
	BMP280_REGISTER_PRESSDATA_XLSB = 0xF9

	QNH = 1020 // Reference pressure in hPa

	BUFSIZE = 256 // Buffer size for reading data from BMP
)

type BMPData struct {
	Temperature float64
	Pressure    float64
	Altitude    float64
	T           time.Time
}

type BMP280 struct {
	i2cbus    embd.I2CBus
	PowerMode byte
	Freq      byte
	Filter    byte
	TempRes   byte
	PresRes   byte

	config  byte
	control byte

	DigT1, DigP1 uint16
	DigT2, DigT3 int16
	DigP2, DigP3 int16
	DigP4, DigP5 int16
	DigP6, DigP7 int16
	DigP8, DigP9 int16

	T_fine int32

	C      <-chan *BMPData
	CBuf   <-chan *BMPData
	cClose chan bool
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
	bmp = new(BMP280)
	bmp.PowerMode = powerMode
	bmp.Freq = freq
	bmp.Filter = filter
	bmp.TempRes = tempRes
	bmp.PresRes = presRes

	bmp.config = (bmp.Freq << 5) + (filter << 2)              // combine bits for config
	bmp.control = (tempRes << 5) + (presRes << 2) + powerMode // combine bits for control

	bmp.i2cbus = embd.NewI2CBus(1)

	// check sensor id 0x58=BMP280
	v, errv := bmp.i2cRead(BMP280_REGISTER_CHIPID)
	if errv != nil {
		err = fmt.Errorf("BMP280: couldn't read ChipID register: %s", errv)
		return
	}
	if v != 0x58 {
		err = fmt.Errorf("BMP280: Wrong ChipID, got %s", v)
		return
	}
	bmp.i2cWrite(BMP280_REGISTER_SOFTRESET, 0xB6) // reset sensor
	time.Sleep(200 * time.Millisecond)
	bmp.i2cWrite(BMP280_REGISTER_CONTROL, bmp.control) //
	time.Sleep(200 * time.Millisecond)
	bmp.i2cWrite(BMP280_REGISTER_CONFIG, bmp.config) //
	time.Sleep(200 * time.Millisecond)

	bmp.ReadCorrectionSettings()
	go bmp.readSensor()
	time.Sleep(50 * time.Millisecond)
	return
}

// Close closes the BMP280
func (bmp *BMP280) Close() {
	// Nothing to do bitwise
	bmp.cClose <- true
}

// Frequency returns the sampling frequency, in Hz
func (bmp *BMP280) Frequency() float64 {
	// freq = 0 (0.5ms), 1 (62.5ms), 2 (125ms), 3 (250ms), [4] (500ms), 5 (1000ms), 6 (2000ms), 7 (4000ms)
	if bmp.Freq == 0 {
		return 2000
	} else if bmp.Freq == 1 {
		return 62.5
	}
	return float64(uint(4000) >> uint(7-bmp.Freq))
}

func (bmp *BMP280) ReadCorrectionSettings() (err error) {
	uRegMap := map[*uint16]byte{
		&bmp.DigT1: BMP280_REGISTER_DIG_T1, &bmp.DigP1: BMP280_REGISTER_DIG_P1,
	}

	regMap := map[*int16]byte{
		&bmp.DigT2: BMP280_REGISTER_DIG_T2, &bmp.DigT3: BMP280_REGISTER_DIG_T3,
		&bmp.DigP2: BMP280_REGISTER_DIG_P2, &bmp.DigP3: BMP280_REGISTER_DIG_P3,
		&bmp.DigP4: BMP280_REGISTER_DIG_P4, &bmp.DigP5: BMP280_REGISTER_DIG_P5,
		&bmp.DigP6: BMP280_REGISTER_DIG_P6, &bmp.DigP7: BMP280_REGISTER_DIG_P7,
		&bmp.DigP8: BMP280_REGISTER_DIG_P8, &bmp.DigP9: BMP280_REGISTER_DIG_P9,
	}

	for p, reg := range uRegMap {
		*p, err = bmp.i2cRead2u(reg)
		if err != nil {
			log.Printf("BMP280 Warning: error reading sensor data from %x", reg)
			*p = 0
			return
		}
	}

	for p, reg := range regMap {
		*p, err = bmp.i2cRead2(reg)
		if err != nil {
			log.Printf("BMP280 Warning: error reading sensor data from %x", reg)
			*p = 0
			return
		}
	}

	return
}

func (bmp *BMP280) readSensor() {
	var (
		raw_temp_msb  uint8
		raw_temp_lsb  uint8
		raw_temp_xlsb uint8
		raw_temp      int32

		raw_press_msb  uint8
		raw_press_lsb  uint8
		raw_press_xlsb uint8
		raw_press      int32

		temp, press, altitude float64
		err                   error

		t time.Time
	)

	regMap := map[*uint8]byte{
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
			Pressure:    press,
			Altitude:    altitude,
			T:           t,
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

			raw_temp = (int32(raw_temp_msb) << 12) + (int32(raw_temp_lsb) << 4) + (int32(raw_temp_xlsb) >> 4) // combine 3 bytes  msb 12 bits left, lsb 4 bits left, xlsb 4 bits right

			raw_press = (int32(raw_press_msb) << 12) + (int32(raw_press_lsb) << 4) + (int32(raw_press_xlsb) >> 4) // combine 3 bytes  msb 12 bits left, lsb 4 bits left, xlsb 4 bits right

			temp = bmp.CalcCompensatedTemp(raw_temp)
			press = bmp.CalcCompensatedTemp(raw_press)
			altitude = bmp.CalcAltitude(temp, press)
		case cC <- makeBMPData(): // Send the latest values
		case cBuf <- makeBMPData():
		case <-bmp.cClose: // Stop the goroutine, ease up on the CPU
			break
		}
	}
}

func (bmp *BMP280) CalcCompensatedTemp(raw_temp int32) (temp float64) {
	var var1, var2, t int32

	var1 = (((raw_temp >> 3) - (int32(bmp.DigT1) << 1)) * int32(bmp.DigT2)) >> 11
	var2 = (((((raw_temp >> 4) - int32(bmp.DigT1)) * ((raw_temp >> 4) - int32(bmp.DigT1))) >> 12) * int32(bmp.DigT3)) >> 14
	bmp.T_fine = var1 + var2
	t = (bmp.T_fine*5 + 128) >> 8
	temp = float64(t) / 100 // Temperature in degC
	return
}

func (bmp *BMP280) CalcCompensatedPress(raw_press int64) (press float64) {
	var var1, var2, p int64

	var1 = int64(bmp.T_fine) - 128000
	var2 = var1 * var1 * int64(bmp.DigP6)
	var2 += (var1 * int64(bmp.DigP5)) << 17
	var2 += int64(bmp.DigP4) << 35
	var1 = ((var1 * var1 * int64(bmp.DigP3)) >> 8) + ((var1 * int64(bmp.DigP2)) << 12)
	var1 = ((int64(1) << 47) + var1) * int64(bmp.DigP1) >> 33
	if var1 == 0 {
		return 0
	}
	p = 1048576 - raw_press
	p = (((p << 31) - var2) * 3125) / var1
	var1 = (int64(bmp.DigP9) * (p >> 13) * (p >> 13)) >> 25
	var2 = (int64(bmp.DigP8) * p) >> 19
	p = ((p + var1 + var2) >> 8) + (int64(bmp.DigP7) << 4)
	press = float64(p) / 25600
	return
}

func (bmp *BMP280) CalcAltitude(temp, press float64) (altitude float64) {
	altitude = 44330.0 * (1.0 - math.Pow(press/(QNH*100), (1.0/5.255)))
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
