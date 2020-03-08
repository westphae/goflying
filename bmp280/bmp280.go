/*
Reference 1: https://github.com/BoschSensortec/BMP280_driver
Reference 2: https://forums.adafruit.com/viewtopic.php?f=19&t=89049
*/

package bmp280

import (
	"fmt"
	"log"
	"math"
	"time"

	"github.com/kidoman/embd"
	_ "github.com/kidoman/embd/host/all"
	_ "github.com/kidoman/embd/host/rpi"
)

const (
	QNH       = 1013.25 // Sea level reference pressure in hPa
	BufSize   = 256 // Buffer size for reading data from BMP
	ReadDelay = time.Millisecond // Delay between chip reading polls
)

type BMPData struct {
	Temperature float64
	Pressure    float64
	T           time.Duration
}

type BMP280 struct {
	i2cbus *embd.I2CBus

	Address byte
	ChipID  byte
	config  byte
	control byte

	t time.Time

	Delay time.Duration

	DigT map[int]int32
	DigP map[int]int64

	T_fine int32

	C      <-chan *BMPData
	CBuf   <-chan *BMPData
	cClose chan bool
}

/*
NewBMP280 returns a BMP280 object with the chosen settings
address is one of bmp280.Address1 or bmp280.Address2
powerMode is one of bmp280.SleepMode, bmp280.ForcedMode, or bmp280.NormalMode
standby is one of the bmp280.StandbyTimeX
filter is one of bmp280.FilterCoeffX
tempRes is one of bmp280.OversampX
presRes is one of bmp280.XMode
*/
func NewBMP280(i2cbus *embd.I2CBus, address, powerMode, standby, filter, tempRes, presRes byte) (bmp *BMP280, err error) {
	bmp = new(BMP280)
	bmp.i2cbus = i2cbus
	bmp.Address = address

	// Make sure we can connect to the chip and read a valid ChipID
	v := make([]byte, 1)
	if errv := bmp.i2cReadBytes(RegisterChipID, v); errv != nil {
		err = fmt.Errorf("BMP280: couldn't find chip at address %x: %s", address, errv)
		return nil, err
	}
	if v[0] != ChipID1 && v[0] != ChipID2 && v[0] != ChipID3 {
		return nil, fmt.Errorf("BMP280: Wrong ChipID, got %x", v)
	}

	bmp.ChipID = v[0]

	bmp.config = (standby << 5) + (filter << 2)               // combine bits for config
	bmp.control = (tempRes << 5) + (presRes << 2) + powerMode // combine bits for control

	bmp.t = time.Now()
	bmp.Delay = delayFromStandby(standby)

	bmp.i2cWrite(RegisterSoftReset, SoftResetCode) // reset sensor

	bmp.i2cWrite(RegisterControl, bmp.control) //
	bmp.i2cWrite(RegisterConfig, bmp.config) //

	bmp.DigT = make(map[int]int32)
	bmp.DigP = make(map[int]int64)
	bmp.ReadCorrectionSettings()

	go bmp.readSensor()
	return
}

// Close closes the BMP280
func (bmp *BMP280) Close() {
	bmp.SetPowerMode(SleepMode)
	bmp.cClose <- true
}

func delayFromStandby(standby byte) (delay time.Duration) {
	if standby == 0 {
		delay = 500 * time.Microsecond
	} else if standby == 1 {
		delay = 62500 * time.Microsecond
	} else {
		delay = time.Duration(int(4000)>>uint(7-standby)) * time.Millisecond
	}
	return
}

func (bmp *BMP280) ReadCorrectionSettings() (err error) {
	var raw []byte = make([]byte, 24)

	errf := bmp.i2cReadBytes(RegisterCompData, raw)
	if errf != nil {
		err = fmt.Errorf("BMP280: Error reading calibration: %s", errf)
	}

	bmp.DigT[1] = int32(raw[1])<<8 + int32(raw[0])
	for i := 1; i < 3; i++ {
		bmp.DigT[i+1] = int32(int16(raw[2*i+1])<<8 + int16(raw[2*i]))
	}

	bmp.DigP[1] = int64(raw[7])<<8 + int64(raw[6])
	for i := 1; i < 9; i++ {
		bmp.DigP[i+1] = int64(int16(raw[2*i+7])<<8 + int16(raw[2*i+6]))
	}

	return
}

func (bmp *BMP280) readSensor() {
	var (
		raw_temp    int32
		raw_press   int64
		temp, press float64
		err         error
		t           time.Time
	)

	raw := make([]byte, 6)

	cC := make(chan *BMPData)
	defer close(cC)
	bmp.C = cC
	cBuf := make(chan *BMPData, BufSize)
	defer close(cBuf)
	bmp.CBuf = cBuf
	bmp.cClose = make(chan bool)
	defer close(bmp.cClose)

	clock := time.NewTicker(bmp.Delay)
	//TODO westphae: use the clock to record actual time instead of a timer
	defer clock.Stop()

	makeBMPData := func() *BMPData {
		d := BMPData{
			Temperature: temp,
			Pressure:    press,
			T:           t.Sub(bmp.t),
		}
		return &d
	}

	// Throw away initial value
	if err = bmp.i2cReadBytes(RegisterPressDataMSB, raw); err != nil {
		log.Printf("bmp280 warning: error reading sensor data: %s", err)
	}

	for {
		select {
		case t = <-clock.C: // Read sensor data:
			err = bmp.i2cReadBytes(RegisterPressDataMSB, raw)
			if err != nil {
				log.Printf("bmp280 warning: error reading sensor data: %s", err)
				continue
			}

			raw_temp = (int32(raw[3]) << 12) + (int32(raw[4]) << 4) + (int32(raw[5]) >> 4) // combine 3 bytes  msb 12 bits left, lsb 4 bits left, xlsb 4 bits right

			raw_press = (int64(raw[0]) << 12) + (int64(raw[1]) << 4) + (int64(raw[2]) >> 4) // combine 3 bytes  msb 12 bits left, lsb 4 bits left, xlsb 4 bits right

			temp = bmp.CalcCompensatedTemp(raw_temp)
			press = bmp.CalcCompensatedPress(raw_press)
		case cC <- makeBMPData(): // Send the latest values
		case cBuf <- makeBMPData():
		case <-bmp.cClose: // Stop the goroutine, ease up on the CPU
			break
		}
	}
}

func (bmp *BMP280) CalcCompensatedTemp(raw_temp int32) (temp float64) {
	var var1, var2, t int32

	var1 = (((raw_temp >> 3) - (bmp.DigT[1] << 1)) * bmp.DigT[2]) >> 11
	var2 = (((((raw_temp >> 4) - bmp.DigT[1]) * ((raw_temp >> 4) - bmp.DigT[1])) >> 12) * bmp.DigT[3]) >> 14
	bmp.T_fine = var1 + var2
	t = (bmp.T_fine*5 + 128) >> 8
	temp = float64(t) / 100 // Temperature in degC
	return
}

func (bmp *BMP280) CalcCompensatedPress(raw_press int64) (press float64) {
	var var1, var2, p int64

	var1 = int64(bmp.T_fine) - 128000
	var2 = var1 * var1 * bmp.DigP[6]
	var2 += (var1 * bmp.DigP[5]) << 17
	var2 += bmp.DigP[4] << 35
	var1 = ((var1 * var1 * bmp.DigP[3]) >> 8) + ((var1 * bmp.DigP[2]) << 12)
	var1 = ((int64(1) << 47) + var1) * bmp.DigP[1] >> 33
	if var1 == 0 {
		return 0
	}
	p = 1048576 - raw_press
	p = (((p << 31) - var2) * 3125) / var1
	var1 = (bmp.DigP[9] * (p >> 13) * (p >> 13)) >> 25
	var2 = (bmp.DigP[8] * p) >> 19
	p = ((p + var1 + var2) >> 8) + (bmp.DigP[7] << 4)
	press = float64(p) / 25600
	return
}

func CalcAltitude(press float64) (altitude float64) {
	altitude = 145366.45 * (1.0 - math.Pow(press/QNH, 0.190284))
	return
}

func (bmp *BMP280) GetPowerMode() (powerMode byte, err error) {
	v := make([]byte, 1)
	if errv := bmp.i2cReadBytes(RegisterControl, v); errv != nil {
		err = fmt.Errorf("bmp280 error: couldn't read power mode: %s", errv)
		return
	}
	powerMode = v[0] & 0x03
	return
}

func (bmp *BMP280) SetPowerMode(powerMode byte) error {
	v := make([]byte, 1)
	if errv := bmp.i2cReadBytes(RegisterControl, v); errv != nil {
		return fmt.Errorf("bmp280 error: couldn't read power mode: %s", errv)
	}
	v[0] = (v[0] & 0xfc) | powerMode

	if errv := bmp.i2cWrite(RegisterControl, v[0]); errv != nil {
		return fmt.Errorf("bmp280 error: couldn't write power mode: %s", errv)
	}
	return nil
}

func (bmp *BMP280) GetOversampPress() (oversampPres byte, err error) {
	v := make([]byte, 1)
	if errv := bmp.i2cReadBytes(RegisterControl, v); errv != nil {
		err = fmt.Errorf("bmp280 error: couldn't read Pressure Oversampling: %s", errv)
		return
	}
	oversampPres = (v[0] & 0x1c) >> 2
	return
}

func (bmp *BMP280) SetOversampPress(oversampPres byte) error {
	v := make([]byte, 1)
	if errv := bmp.i2cReadBytes(RegisterControl, v); errv != nil {
		return fmt.Errorf("bmp280 error: couldn't read Pressure Oversampling: %s", errv)
	}
	v[0] = (v[0] & 0xe3) | (oversampPres << 2)

	time.Sleep(bmp.Delay)
	if errv := bmp.i2cWrite(RegisterControl, v[0]); errv != nil {
		return fmt.Errorf("bmp280 error: couldn't write Pressure Oversampling: %s", errv)
	}
	return nil
}

func (bmp *BMP280) GetOversampTemp() (oversampTemp byte, err error) {
	v := make([]byte, 1)
	if errv := bmp.i2cReadBytes(RegisterControl, v); errv != nil {
		err = fmt.Errorf("bmp280 error: couldn't read Temperature Oversampling: %s", errv)
		return
	}
	oversampTemp = (v[0] & 0xe0) >> 5
	return
}

func (bmp *BMP280) SetOversampTemp(oversampTemp byte) error {
	v := make([]byte, 1)
	if errv := bmp.i2cReadBytes(RegisterControl, v); errv != nil {
		return fmt.Errorf("bmp280 error: couldn't read Temperature Oversampling: %s", errv)
	}
	v[0] = (v[0] & 0x1f) | (oversampTemp << 5)

	time.Sleep(bmp.Delay)
	if errv := bmp.i2cWrite(RegisterControl, v[0]); errv != nil {
		return fmt.Errorf("bmp280 error: couldn't write Temperature Oversampling: %s", errv)
	}
	return nil
}

func (bmp *BMP280) GetFilterCoeff() (filterCoeff byte, err error) {
	v := make([]byte, 1)
	if errv := bmp.i2cReadBytes(RegisterConfig, v); errv != nil {
		err = fmt.Errorf("bmp280 error: couldn't read Filter Coefficient: %s", errv)
		return
	}
	filterCoeff = (v[0] & 0x1c) >> 2
	return
}

func (bmp *BMP280) SetFilterCoeff(filterCoeff byte) error {
	v := make([]byte, 1)
	if errv := bmp.i2cReadBytes(RegisterConfig, v); errv != nil {
		return fmt.Errorf("bmp280 error: couldn't read Filter Coefficient: %s", errv)
	}
	v[0] = (v[0] & 0xe3) | (filterCoeff << 2)

	if errv := bmp.i2cWrite(RegisterConfig, v[0]); errv != nil {
		return fmt.Errorf("bmp280 error: couldn't write Filter Coefficient: %s", errv)
	}
	return nil
}

func (bmp *BMP280) GetStandbyTime() (standbyTime byte, err error) {
	v := make([]byte, 1)
	if errv := bmp.i2cReadBytes(RegisterConfig, v); errv != nil {
		err = fmt.Errorf("bmp280 error: couldn't read Standby Time: %s", errv)
		return
	}
	standbyTime = (v[0] & 0xe0) >> 5
	return
}

func (bmp *BMP280) SetStandbyTime(standbyTime byte) error {
	v := make([]byte, 1)
	if errv := bmp.i2cReadBytes(RegisterConfig, v); errv != nil {
		return fmt.Errorf("bmp280 error: couldn't read Standby Time: %s", errv)
	}
	v[0] = (v[0] & 0x1f) | (standbyTime << 5)

	if errv := bmp.i2cWrite(RegisterConfig, v[0]); errv != nil {
		return fmt.Errorf("bmp280 error: couldn't write Standby Time: %s", errv)
	}
	bmp.Delay = delayFromStandby(standbyTime)
	return nil
}

func (bmp *BMP280) i2cWrite(register, value byte) (err error) {
	if errWrite := (*bmp.i2cbus).WriteByteToReg(bmp.Address, register, value); errWrite != nil {
		err = fmt.Errorf("bmp280 error writing %X to %X: %s\n",
			value, register, errWrite)
	}
	time.Sleep(ReadDelay)
	return
}

func (bmp *BMP280) i2cReadBytes(register byte, value []byte) (err error) {
	errRead := (*bmp.i2cbus).ReadFromReg(bmp.Address, register, value)
	if errRead != nil {
		err = fmt.Errorf("bmp280 error reading from %X: %s", register, errRead.Error())
	}
	return
}
