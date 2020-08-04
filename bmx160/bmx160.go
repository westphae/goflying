package bmx160

import (
	"encoding/json"
	"errors"
	"fmt"
	"log"
	"math"
	"os"
	"time"
	"github.com/kidoman/embd"
	_ "github.com/kidoman/embd/host/all" // Empty import needed to initialize embd library.
	_ "github.com/kidoman/embd/host/rpi" // Empty import needed to initialize embd library.
)

const (
	bufSize         = 25 // Size of buffer storing instantaneous sensor values
	scaleMag        = 9830.0 / 65536
	calDataLocation = "/etc/bmx160cal.json"
)

// MPUData contains all the values measured by an BMX160.
type MPUData struct {
	G1, G2, G3        float64
	A1, A2, A3        float64
	M1, M2, M3        float64
	Temp              float64
	GAError, MagError error
	N, NM             int
	T, TM             time.Time
	DT, DTM           time.Duration
}

type mpuCalData struct {
	A01, A02, A03    float64 // Accelerometer hardware bias
	G01, G02, G03    float64 // Gyro hardware bias
	M01, M02, M03    float64 // Magnetometer hardware bias
	Ms11, Ms12, Ms13 float64 // Magnetometer rescaling matrix
	Ms21, Ms22, Ms23 float64 // (Only diagonal is used currently)
	Ms31, Ms32, Ms33 float64
}

func (d *mpuCalData) reset() {
	d.Ms11 = 1
	d.Ms22 = 1
	d.Ms33 = 1
}

func (d *mpuCalData) save() {
	fd, err := os.OpenFile(calDataLocation, os.O_CREATE|os.O_WRONLY|os.O_TRUNC, os.FileMode(0644))
	if err != nil {
		log.Printf("BMX160: Error saving calibration data to %s: %s", calDataLocation, err.Error())
		return
	}
	defer fd.Close()
	calData, err := json.Marshal(d)
	if err != nil {
		log.Printf("BMX160: Error marshaling calibration data: %s", err)
		return
	}
	fd.Write(calData)
}

func (d *mpuCalData) load() (err error) {
	//d.M01 = 1638.0
	//d.M02 = -589.0
	//d.M03 = -2153.0
	//d.Ms11 = 0.00031969309462915601
	//d.Ms22 = 0.00035149384885764499
	//d.Ms33 = 0.00028752156411730879
	//d.save()
	//return
	errstr := "BMX160: Error reading calibration data from %s: %s"
	fd, rerr := os.Open(calDataLocation)
	if rerr != nil {
		err = fmt.Errorf(errstr, calDataLocation, rerr.Error())
		return
	}
	defer fd.Close()
	buf := make([]byte, 1024)
	count, rerr := fd.Read(buf)
	if rerr != nil {
		err = fmt.Errorf(errstr, calDataLocation, rerr.Error())
		return
	}
	rerr = json.Unmarshal(buf[0:count], d)
	if rerr != nil {
		err = fmt.Errorf(errstr, calDataLocation, rerr.Error())
		return
	}
	return
}

/*
BMX160 represents a Bosch BMX160 9DoF chip.
falsfalse communication is via channels.
*/
type BMX160 struct {
	i2cbus                embd.I2CBus
	scaleGyro, scaleAccel float64 // Max sensor reading for value 2**15-1
	sampleRate            int
	enableMag             bool
	mpuCalData
	mcal1, mcal2, mcal3 float64         // Hardware magnetometer calibration values, uT
	C                   <-chan *MPUData // Current instantaneous sensor values
	CAvg                <-chan *MPUData // Average sensor values (since CAvg last read)
	CBuf                <-chan *MPUData // Buffer of instantaneous sensor values
	cClose              chan bool       // Turn off MPU polling
}

/*
NewBMX160 creates a new BMX160 object according to the supplied parameters.  If there is no BMX160 available or there
is an error creating the object, an error is returned.
do we need to load calibration data?
*/
	//if err := mpu.mpuCalData.load(); err != nil {
	//	mpu.mpuCalData.reset()
	//}

//do we need synchronization of data
//ToDo Selftest of sensors, tapping to set zero values
//after setting sampling rate I should read the data to clear any hickups per datasheet
//temperature corrections
//Implement reading the magnetometer
func NewBMX160(i2cbus *embd.I2CBus, sensitivityGyro, sensitivityAccel, sampleRate int, enableMag bool, applyHWOffsets bool) (*BMX160, error) {
	var mpu = new(BMX160)
	mpu.i2cbus = *i2cbus
	mpu.sampleRate = sampleRate
	mpu.enableMag = enableMag
        
        id, err := mpu.GetChipID()
        if id != 0xd8 {
           log.Printf("chip is not BMX160! Chip ID is %x\n", id)
           return nil, errors.New("chip is not BMX160!")
	}
        log.Printf("found IMU chip BMX160!\n")

        if id != BMI160_CHIP_ID {
                return nil, errors.New("IMU is not BMX160")
        }
        
        //soft reset of IMU
        if err := mpu.i2cWrite(BMI160_COMMAND_REG_ADDR,BMI160_SOFT_RESET_CMD); err != nil { 
               return nil, errors.New("Error resetting BMX160")
        }

        //set all instruments: accelerometer, gyroscope, and magnetometer into normal power mode
        if err := mpu.i2cWrite(BMI160_COMMAND_REG_ADDR,BMI160_ACCEL_NORMAL_MODE); err != nil { 
               return nil, errors.New("Error setting accelerometer in normal mode")
        }
        time.Sleep(5 * time.Millisecond) // wait time to switch to normal mode


        if err := mpu.i2cWrite(BMI160_COMMAND_REG_ADDR,BMI160_GYRO_NORMAL_MODE); err != nil { 
               return nil, errors.New("Error setting gyroscope into normal mode")
        }
        time.Sleep(90 * time.Millisecond) // wait time to switch to mormal mode

        if err := mpu.i2cWrite(BMI160_COMMAND_REG_ADDR,BMI160_AUX_NORMAL_MODE); err != nil { 
               return nil, errors.New("Error setting magnetometer into normal mode")
        }
        time.Sleep(1 * time.Millisecond) // wait time to switch to normal mode

	value, err := mpu.i2cRead(BMI160_PMU_STATUS_ADDR)
	if err != nil {
		return nil, errors.New("Error reading BMX160 power mode status")
	}

        if value != 0x15 {
                return nil, errors.New("BMX160 couldn't be set to normal power mode")
        }
        

        //configuring gyro
	if err := mpu.SetSampleRateGyro(mpu.sampleRate); err != nil {
		return nil, err
	}

	if err := mpu.SetGyroSensitivity(sensitivityGyro); err != nil {
		log.Println(err)
	}


        //configuring accelerometer
	if err := mpu.SetSampleRateAccel(mpu.sampleRate); err != nil {
		return nil, err
	}

	if err := mpu.SetAccelSensitivity(sensitivityAccel); err != nil {
		log.Println(err)
	}
        

        //Turning FIFO off
	if err := mpu.i2cWrite(BMI160_FIFO_CONFIG_0_ADDR, 0x80); err != nil {
		return nil, errors.New("BMX160 Error: couldn't disable FIFO")
	}
	value, err = mpu.i2cRead(BMI160_FIFO_CONFIG_0_ADDR)
        if  err != nil {
		return nil, errors.New("Error reading BMX160 FIFO Config address 0")
	}
        if value != 0x80 {
                return nil, errors.New("BMX160 FIFO ADDR 0 is not configure in the default way")
        }

	if err := mpu.i2cWrite(BMI160_FIFO_CONFIG_1_ADDR, 0x10); err != nil {
		return nil, errors.New("BMX160 Error: couldn't disable FIFO")
	}
	value, err = mpu.i2cRead(BMI160_FIFO_CONFIG_1_ADDR)
        if  err != nil {
		return nil, errors.New("Error reading BMX160 FIFO Config address 0")
	}
        if value != 0x10 {
                return nil, errors.New("BMX160 FIFO ADDR 1 is not configure in the default way")
        }
        //fmt.Printf("%x\n", value)
        


	// Turn off interrupts
	if err := mpu.i2cWrite(BMI160_INT_ENABLE_0_ADDR, 0x00); err != nil {
		return nil, errors.New("BMX160 Error: couldn't disable interrupts")
	}
	if err := mpu.i2cWrite(BMI160_INT_ENABLE_1_ADDR, 0x00); err != nil {
		return nil, errors.New("BMX160 Error: couldn't disable interrupts")
	}
	if err := mpu.i2cWrite(BMI160_INT_ENABLE_2_ADDR, 0x00); err != nil {
		return nil, errors.New("BMX160 Error: couldn't disable interrupts")
	}

       mpu.FastOffsetCompensation()

       mpu.EnableOffsetCompensation()
///////////////////////////////////////////////////////////////////////////////////////////////

	// Set up magnetometer
	if mpu.enableMag {
		if err := mpu.ReadMagCalibration(); err != nil {
			return nil, errors.New("Error reading calibration from magnetometer")
		}

		// Set up AK8963 master mode, master clock and ES bit
		if err := mpu.i2cWrite(MPUREG_I2C_MST_CTRL, 0x40); err != nil {
			return nil, errors.New("Error setting up AK8963")
		}
		// Slave 0 reads from AK8963
		if err := mpu.i2cWrite(MPUREG_I2C_SLV0_ADDR, BIT_I2C_READ|AK8963_I2C_ADDR); err != nil {
			return nil, errors.New("Error setting up AK8963")
		}
		// Compass reads start at this register
		if err := mpu.i2cWrite(MPUREG_I2C_SLV0_REG, AK8963_ST1); err != nil {
			return nil, errors.New("Error setting up AK8963")
		}
		// Enable 8-byte reads on slave 0
		if err := mpu.i2cWrite(MPUREG_I2C_SLV0_CTRL, BIT_SLAVE_EN|8); err != nil {
			return nil, errors.New("Error setting up AK8963")
		}
		// Slave 1 can change AK8963 measurement mode
		if err := mpu.i2cWrite(MPUREG_I2C_SLV1_ADDR, AK8963_I2C_ADDR); err != nil {
			return nil, errors.New("Error setting up AK8963")
		}
		if err := mpu.i2cWrite(MPUREG_I2C_SLV1_REG, AK8963_CNTL1); err != nil {
			return nil, errors.New("Error setting up AK8963")
		}
		// Enable 1-byte reads on slave 1
		if err := mpu.i2cWrite(MPUREG_I2C_SLV1_CTRL, BIT_SLAVE_EN|1); err != nil {
			return nil, errors.New("Error setting up AK8963")
		}
		// Set slave 1 data
		if err := mpu.i2cWrite(MPUREG_I2C_SLV1_DO, AKM_SINGLE_MEASUREMENT); err != nil {
			return nil, errors.New("Error setting up AK8963")
		}
		// Triggers slave 0 and 1 actions at each sample
		if err := mpu.i2cWrite(MPUREG_I2C_MST_DELAY_CTRL, 0x03); err != nil {
			return nil, errors.New("Error setting up AK8963")
		}

		// Set AK8963 sample rate to same as gyro/accel sample rate, up to max
		var ak8963Rate byte
		if mpu.sampleRate < AK8963_MAX_SAMPLE_RATE {
			ak8963Rate = 0
		} else {
			ak8963Rate = byte(mpu.sampleRate/AK8963_MAX_SAMPLE_RATE - 1)
		}

		// Not so sure of this one--I2C Slave 4??!
		if err := mpu.i2cWrite(MPUREG_I2C_SLV4_CTRL, ak8963Rate); err != nil {
			return nil, errors.New("Error setting up AK8963")
		}

		time.Sleep(100 * time.Millisecond) // Make sure mag is ready
	}
/*

// set that in initialization
	if mpu.sampleRate > 100 {
		magSampleRate = 100
	} else {
		magSampleRate = mpu.sampleRate
	}
*/


	go mpu.readSensors()

	// Give the IMU time to fully initialize and then clear out any bad values from the averages.
	time.Sleep(500 * time.Millisecond) // Make sure it's ready
	<-mpu.CAvg

	return mpu, nil
}

// readSensors polls the gyro, accelerometer and magnetometer sensors as well as the die temperature.
// Communication is via channels.
func (mpu *BMX160) readSensors() {
	var (
		g1, g2, g3, a1, a2, a3, m1, m2, m3, m4, tmp int16   // Current values
		avg1, avg2, avg3, ava1, ava2, ava3, avtmp   float64 // Accumulators for averages
		avm1, avm2, avm3                            int32
		n, nm                                       float64
		gaError, magError                           error
		t0, t, t0m, tm                              time.Time
		magSampleRate                               int
		curdata                                     *MPUData
	)

	acRegMap := map[*int16]byte{
		&g2: BMI160_GYRO_DATA_ADDR, &g1: BMI160_GYRO_DATA_ADDR+0x02, &g3: BMI160_GYRO_DATA_ADDR+0x04,
		&a2: BMI160_ACCEL_DATA_ADDR, &a1: BMI160_ACCEL_DATA_ADDR+0x02, &a3: BMI160_ACCEL_DATA_ADDR+0x04,
		&tmp: 0x20,
	}

/*
	acRegMap := map[*int16]byte{
		&g1: BMI160_GYRO_DATA_ADDR, &g2: BMI160_GYRO_DATA_ADDR+0x02, &g3: BMI160_GYRO_DATA_ADDR+0x04,
		&a1: BMI160_ACCEL_DATA_ADDR, &a2: BMI160_ACCEL_DATA_ADDR+0x02, &a3: BMI160_ACCEL_DATA_ADDR+0x04,
		&tmp: 0x20,
	}
*/
	magRegMap := map[*int16]byte{
		&m1: BMI160_AUX_DATA_ADDR, &m2: BMI160_AUX_DATA_ADDR+0x02, &m3: BMI160_AUX_DATA_ADDR+0x04, &m4: BMI160_AUX_DATA_ADDR+0x06,
	}

	cC := make(chan *MPUData)
	defer close(cC)
	mpu.C = cC
	cAvg := make(chan *MPUData)
	defer close(cAvg)
	mpu.CAvg = cAvg
	cBuf := make(chan *MPUData, bufSize)
	defer close(cBuf)
	mpu.CBuf = cBuf
	mpu.cClose = make(chan bool)
	defer close(mpu.cClose)

	clock := time.NewTicker(time.Duration(int(1000.0/float32(mpu.sampleRate)+0.5)) * time.Millisecond)
	//TODO westphae: use the clock to record actual time instead of a timer
	defer clock.Stop()

	clockMag := time.NewTicker(time.Duration(int(1000.0/float32(magSampleRate)+0.5)) * time.Millisecond)
	t0 = time.Now()
	t0m = time.Now()


	makeMPUData := func() *MPUData {
		mm1 := float64(m1)*mpu.mcal1 - mpu.M01
		mm2 := float64(m2)*mpu.mcal2 - mpu.M02
		mm3 := float64(m3)*mpu.mcal3 - mpu.M03
		d := MPUData{
			G1:      -1.0 * (float64(g1) - mpu.G01) * mpu.scaleGyro,
			G2:      -1.0 * (float64(g2) - mpu.G02) * mpu.scaleGyro,
			G3:      -1.0 * (float64(g3) - mpu.G03) * mpu.scaleGyro,
			A1:      -1.0 * (float64(a1) - mpu.A01) * mpu.scaleAccel,
			A2:      -1.0 * (float64(a2) - mpu.A02) * mpu.scaleAccel,
			A3:      -1.0 * (float64(a3) - mpu.A03) * mpu.scaleAccel,
			M1:      mpu.Ms11*mm1 + mpu.Ms12*mm2 + mpu.Ms13*mm3,
			M2:      mpu.Ms21*mm1 + mpu.Ms22*mm2 + mpu.Ms23*mm3,
			M3:      mpu.Ms31*mm1 + mpu.Ms32*mm2 + mpu.Ms33*mm3,
			Temp:    float64(tmp)*0.001938 + 23.0,
			GAError: gaError, MagError: magError,
			N: 1, NM: 1,
			T: t, TM: tm,
			DT: time.Duration(0), DTM: time.Duration(0),
		}
		if gaError != nil {
			d.N = 0
		}
		if magError != nil {
			d.NM = 0
		}
		return &d
	}

	makeAvgMPUData := func() *MPUData {
		mm1 := float64(avm1)*mpu.mcal1/nm - mpu.M01
		mm2 := float64(avm2)*mpu.mcal2/nm - mpu.M02
		mm3 := float64(avm3)*mpu.mcal3/nm - mpu.M03
		d := MPUData{}
		if n > 0.5 {
			d.G1 = -1.0 * (avg1/n - mpu.G01) * mpu.scaleGyro
			d.G2 = -1.0 * (avg2/n - mpu.G02) * mpu.scaleGyro
			d.G3 = -1.0 * (avg3/n - mpu.G03) * mpu.scaleGyro
			d.A1 = -1.0 * (ava1/n - mpu.A01) * mpu.scaleAccel
			d.A2 = -1.0 * (ava2/n - mpu.A02) * mpu.scaleAccel
			d.A3 = -1.0 * (ava3/n - mpu.A03) * mpu.scaleAccel
			d.Temp = (float64(avtmp)/n)*0.001938 + 23.0
			d.N = int(n + 0.5)
			d.T = t
			d.DT = t.Sub(t0)
		} else {
			d.GAError = errors.New("BMX160 Error: No new accel/gyro values")
		}
		if nm > 0 {
			d.M1 = mpu.Ms11*mm1 + mpu.Ms12*mm2 + mpu.Ms13*mm3
			d.M2 = mpu.Ms21*mm1 + mpu.Ms22*mm2 + mpu.Ms23*mm3
			d.M3 = mpu.Ms31*mm1 + mpu.Ms32*mm2 + mpu.Ms33*mm3
			d.NM = int(nm + 0.5)
			d.TM = tm
			d.DTM = t.Sub(t0m)
		} else {
			d.MagError = errors.New("BMX160 Error: No new magnetometer values")
		}
		return &d
	}

	for {
		select {
		case t = <-clock.C: // Read accel/gyro data:
			for p, reg := range acRegMap {
				*p, gaError = mpu.i2cRead2(reg)
				if gaError != nil {
					log.Println("BMX160 Warning: error reading gyro/accel")
				}
			}

			curdata = makeMPUData()
			// Update accumulated values and increment count of gyro/accel readings
			avg1 += float64(g1)
			avg2 += float64(g2)
			avg3 += float64(g3)
			ava1 += float64(a1)
			ava2 += float64(a2)
			ava3 += float64(a3)
			avtmp += float64(tmp)
			avm1 += int32(m1)
			avm2 += int32(m2)
			avm3 += int32(m3)
			n++
			select {
			case cBuf <- curdata: // We update the buffer every time we read a new value.
			default: // If buffer is full, remove oldest value and put in newest.
				<-cBuf
				cBuf <- curdata
			}
		case tm = <-clockMag.C: // Read magnetometer data:
			if mpu.enableMag {
				// Set AK8963 to slave0 for reading
				if err := mpu.i2cWrite(MPUREG_I2C_SLV0_ADDR, AK8963_I2C_ADDR|READ_FLAG); err != nil {
					log.Printf("MPU9250 Error: couldn't set AK8963 address for reading: %s", err.Error())
				}
				//I2C slave 0 register address from where to begin data transfer
				if err := mpu.i2cWrite(MPUREG_I2C_SLV0_REG, AK8963_HXL); err != nil {
					log.Printf("MPU9250 Error: couldn't set AK8963 read register: %s", err.Error())
				}
				//Tell AK8963 that we will read 7 bytes
				if err := mpu.i2cWrite(MPUREG_I2C_SLV0_CTRL, 0x87); err != nil {
					log.Printf("MPU9250 Error: couldn't communicate with AK8963: %s", err.Error())
				}

				// Read the actual data
				for p, reg := range magRegMap {
					*p, magError = mpu.i2cRead2(reg)
					if magError != nil {
						log.Println("MPU9250 Warning: error reading magnetometer")
					}
				}

				// Test validity of magnetometer data
				if (byte(m1&0xFF)&AKM_DATA_READY) == 0x00 && (byte(m1&0xFF)&AKM_DATA_OVERRUN) != 0x00 {
					log.Println("MPU9250 mag data not ready or overflow")
					log.Printf("MPU9250 m1 LSB: %X\n", byte(m1&0xFF))
					continue // Don't update the accumulated values
				}

				if (byte((m4>>8)&0xFF) & AKM_OVERFLOW) != 0x00 {
					log.Println("MPU9250 mag data overflow")
					log.Printf("MPU9250 m4 MSB: %X\n", byte((m1>>8)&0xFF))
					continue // Don't update the accumulated values
				}

				// Update values and increment count of magnetometer readings
				avm1 += int32(m1)
				avm2 += int32(m2)
				avm3 += int32(m3)
				nm++
			}
		case cC <- curdata: // Send the latest values
		case cAvg <- makeAvgMPUData(): // Send the averages
			avg1, avg2, avg3 = 0, 0, 0
			ava1, ava2, ava3 = 0, 0, 0
			avm1, avm2, avm3 = 0, 0, 0
			avtmp = 0
			n, nm = 0, 0
			t0, t0m = t, tm
		case <-mpu.cClose: // Stop the goroutine, ease up on the CPU
			break
		}
	}
}

// CloseMPU stops the driver from reading the MPU.
//TODO westphae: need a way to start it going again!
func (mpu *BMX160) CloseMPU() {
	mpu.cClose <- true
}

// SetSampleRate changes the sampling rate of the MPU.
//we use the OSR4 filter, which results in a bandwidth four times 
//lower than the 3dB cutoff frequencies listed in the data sheet
func (mpu *BMX160) SetSampleRateGyro(rate int) (err error) {

	var odr byte
	switch {
	case rate >= 3200:
		odr = BMI160_GYRO_ODR_3200HZ
	case rate >= 1600:
		odr = BMI160_GYRO_ODR_1600HZ
	case rate >= 800:
		odr = BMI160_GYRO_ODR_800HZ
	case rate >= 400:
		odr = BMI160_GYRO_ODR_400HZ
	case rate >= 200:
		odr = BMI160_GYRO_ODR_200HZ
	case rate >= 100:
		odr = BMI160_GYRO_ODR_100HZ
	case rate >= 50:
		odr = BMI160_GYRO_ODR_50HZ
	default:
		odr = BMI160_GYRO_ODR_25HZ
	}

	errWrite := mpu.i2cWrite(BMI160_GYRO_CONFIG_ADDR, odr)
	if errWrite != nil {
		err = fmt.Errorf("BMX160 Error: couldn't set Gyro Sample Rate: %s", errWrite.Error())
	}
        
	value, errWrite := mpu.i2cRead(BMI160_ERROR_REG_ADDR)
        if value != 0 {
	   err = fmt.Errorf("Error setting sampling rate. Value not allowed read the data sheet on gyr_conf: %x", value)
        }
	return
}

// SetSampleRate changes the sampling rate of the MPU.
//we use the OSR4 filter, which results in a bandwidth four times 
//lower than the 3dB cutoff frequencies listed in the data sheet
func (mpu *BMX160) SetSampleRateAccel(rate int) (err error) {

	var odr byte
	switch {
	case rate >= 1600:
		odr = BMI160_ACCEL_ODR_1600HZ
	case rate >= 800:
		odr =  BMI160_ACCEL_ODR_800HZ
	case rate >= 400:
		odr =  BMI160_ACCEL_ODR_400HZ
	case rate >= 200:
		odr =  BMI160_ACCEL_ODR_200HZ
	case rate >= 100:
		odr =  BMI160_ACCEL_ODR_100HZ
	case rate >= 50:
		odr =  BMI160_ACCEL_ODR_50HZ
	case rate >= 25:
		odr =  BMI160_ACCEL_ODR_25HZ
	default:
		odr = BMI160_ACCEL_ODR_12_5HZ
	}

	errWrite := mpu.i2cWrite(BMI160_ACCEL_CONFIG_ADDR, odr)
	if errWrite != nil {
		err = fmt.Errorf("BMX160 Error: couldn't set accelerometer Sample Rate: %s", errWrite.Error())
	}
        
	value, errWrite := mpu.i2cRead(BMI160_ERROR_REG_ADDR)
        if value != 0 {
	   err = fmt.Errorf("Error setting sampling rate for accelerometer. Value not allowed read the data sheet on gyr_conf: %x", value)
        }
	return
}

//GetChip ID
func (mpu *BMX160) GetChipID() (id byte, err error) {
	id, errRead := mpu.i2cRead(BMI160_CHIP_ID_ADDR)
	if errRead != nil {
		return 0, errors.New("Error reading BMX160 chip ID")
	}

	return
}


// SampleRate returns the current sample rate of the BMX160, in Hz.
func (mpu *BMX160) SampleRate() int {
	return mpu.sampleRate
}

// MagEnabled returns whether or not the magnetometer is being read.
func (mpu *BMX160) MagEnabled() bool {
	return mpu.enableMag
}

// SetGyroSensitivity sets the gyro sensitivity of the MPU9250; it must be one of the following values:
// 250, 500, 1000, 2000 (all in °/s).
func (mpu *BMX160) SetGyroSensitivity(sensitivityGyro int) (err error) {
	var sensGyro byte

	switch sensitivityGyro {
	case 2000:
		sensGyro = BMI160_GYRO_RANGE_2000_DPS
		mpu.scaleGyro = 2000.0 / float64(math.MaxInt16)
	case 1000:
		sensGyro = BMI160_GYRO_RANGE_1000_DPS 
		mpu.scaleGyro = 1000.0 / float64(math.MaxInt16)
	case 500:
		sensGyro = BMI160_GYRO_RANGE_500_DPS 
		mpu.scaleGyro = 500.0 / float64(math.MaxInt16)
	case 250:
		sensGyro = BMI160_GYRO_RANGE_250_DPS 
		mpu.scaleGyro = 250.0 / float64(math.MaxInt16)
	case 125:
		sensGyro = BMI160_GYRO_RANGE_125_DPS 
		mpu.scaleGyro = 125.0 / float64(math.MaxInt16)
	default:
		err = fmt.Errorf("BMX160 Error: %d is not a valid gyro sensitivity", sensitivityGyro)
	}

	if errWrite := mpu.i2cWrite(BMI160_GYRO_RANGE_ADDR, sensGyro); errWrite != nil {
		err = errors.New("BMX160: couldn't set gyro sensitivity")
	}

	return
}

// SetAccelSensitivity sets the accelerometer sensitivity of the BMX160; it must be one of the following values:
// 2, 4, 8, 16, all in G (gravity).
func (mpu *BMX160) SetAccelSensitivity(sensitivityAccel int) (err error) {
	var sensAccel byte

	switch sensitivityAccel {
	case 16:
		sensAccel = BMI160_ACCEL_RANGE_16G
		mpu.scaleAccel = 16.0 / float64(math.MaxInt16)
	case 8:
		sensAccel = BMI160_ACCEL_RANGE_8G
		mpu.scaleAccel = 8.0 / float64(math.MaxInt16)
	case 4:
		sensAccel = BMI160_ACCEL_RANGE_4G
		mpu.scaleAccel = 4.0 / float64(math.MaxInt16)
	case 2:
		sensAccel = BMI160_ACCEL_RANGE_2G
		mpu.scaleAccel = 2.0 / float64(math.MaxInt16)
	default:
		err = fmt.Errorf("BMX160 Error: %d is not a valid accel sensitivity", sensitivityAccel)
	}

	if errWrite := mpu.i2cWrite(BMI160_ACCEL_RANGE_ADDR, sensAccel); errWrite != nil {
		err = errors.New("BMX160 Error: couldn't set accel sensitivity")
	}

	return
}

// ReadAccelBias reads the bias accelerometer value stored on the chip.
// These values are set at the factory.
func (mpu *BMX160) ReadAccelBias(sensitivityAccel int) error {
	a0x, err := mpu.i2cRead2(MPUREG_XA_OFFSET_H)
	if err != nil {
		return errors.New("BMX160 Error: ReadAccelBias error reading chip")
	}
	a0y, err := mpu.i2cRead2(MPUREG_YA_OFFSET_H)
	if err != nil {
		return errors.New("BMX160 Error: ReadAccelBias error reading chip")
	}
	a0z, err := mpu.i2cRead2(MPUREG_ZA_OFFSET_H)
	if err != nil {
		return errors.New("BMX160 Error: ReadAccelBias error reading chip")
	}

	switch sensitivityAccel {
	case 16:
		mpu.A01 = float64(a0x >> 1)
		mpu.A02 = float64(a0y >> 1)
		mpu.A03 = float64(a0z >> 1)
	case 8:
		mpu.A01 = float64(a0x)
		mpu.A02 = float64(a0y)
		mpu.A03 = float64(a0z)
	case 4:
		mpu.A01 = float64(a0x << 1)
		mpu.A02 = float64(a0y << 1)
		mpu.A03 = float64(a0z << 1)
	case 2:
		mpu.A01 = float64(a0x << 2)
		mpu.A02 = float64(a0y << 2)
		mpu.A03 = float64(a0z << 2)
	default:
		return fmt.Errorf("BMX160 Error: %d is not a valid acceleration sensitivity", sensitivityAccel)
	}

	return nil
}

// Fast Offset Compensation of gyroscope and accelerometer
func (mpu *BMX160) FastOffsetCompensation() error {


	if err := mpu.i2cWrite(BMI160_FOC_CONF_ADDR,BMI160_GYRO_FOC_EN_MSK|BMI160_ACCEL_FOC_X_CONF_MSK|BMI160_ACCEL_FOC_Y_CONF_MSK|BMI160_ACCEL_FOC_Z_CONF_MSK); err != nil {
		return errors.New("BMX160 Error: couldn't configure fast offset calibration")
	}

        //trigger FOC
        if err := mpu.i2cWrite(BMI160_COMMAND_REG_ADDR,BMI160_START_FOC_CMD); err != nil { 
               return errors.New("Error start fast offset compensation BMX160")
        }
        time.Sleep(500 * time.Millisecond) // wait time for FOC to finish

	stat, err := mpu.i2cRead(BMI160_STATUS_ADDR)
	if err != nil {
		return errors.New("BMX160 Error: Reading Status Byte after FOC")
	}
        if stat&BMI160_FOC_STATUS_MSK != 0x8 {
		return errors.New("BMX160 Error: Couldn't complete FOC")
        }

	return nil
}

// Enable Offset Compensation for gyroscope and accelerometer
func (mpu *BMX160) EnableOffsetCompensation() error {

       //Get the OFFSET byte that also holds the 9:8 bits of the gyro offsets, we don't want to disturb those bits
       stat, err := mpu.i2cRead(BMI160_OFFSET_CONF_ADDR)
	if err != nil {
		return errors.New("BMX160 Error: Reading Offset config byte")
	}

       if err := mpu.i2cWrite(BMI160_OFFSET_CONF_ADDR,stat|BMI160_GYRO_OFFSET_EN_MSK|BMI160_ACCEL_OFFSET_EN_MSK); err != nil {
		return errors.New("BMX160 Error: couldn't enable to offset")
	}

	stat, err = mpu.i2cRead(BMI160_OFFSET_CONF_ADDR)
	if err != nil {
		return errors.New("BMX160 Error: Reading Offset config byte")
	}


        if stat&(BMI160_GYRO_OFFSET_EN_MSK|BMI160_ACCEL_OFFSET_EN_MSK) != 0xC0 {
		return errors.New("BMX160 Error: Couldn't enable offset compensation")
        }

	return nil
}


// ReadMagCalibration reads the magnetometer bias values stored on the chpi.
// These values are set at the factory.
func (mpu *BMX160) ReadMagCalibration() error {
	// Enable bypass mode
	var tmp uint8
	var err error
	tmp, err = mpu.i2cRead(MPUREG_USER_CTRL)
	if err != nil {
		return errors.New("ReadMagCalibration error reading chip")
	}
	if err = mpu.i2cWrite(MPUREG_USER_CTRL, tmp & ^BIT_AUX_IF_EN); err != nil {
		return errors.New("ReadMagCalibration error reading chip")
	}
	time.Sleep(3 * time.Millisecond)
	if err = mpu.i2cWrite(MPUREG_INT_PIN_CFG, BIT_BYPASS_EN); err != nil {
		return errors.New("ReadMagCalibration error reading chip")
	}

	// Prepare for getting sensitivity data from AK8963
	//Set the I2C slave address of AK8963
	if err = mpu.i2cWrite(MPUREG_I2C_SLV0_ADDR, AK8963_I2C_ADDR); err != nil {
		return errors.New("ReadMagCalibration error reading chip")
	}
	// Power down the AK8963
	if err = mpu.i2cWrite(MPUREG_I2C_SLV0_CTRL, AK8963_CNTL1); err != nil {
		return errors.New("ReadMagCalibration error reading chip")
	}
	// Power down the AK8963
	if err = mpu.i2cWrite(MPUREG_I2C_SLV0_DO, AKM_POWER_DOWN); err != nil {
		return errors.New("ReadMagCalibration error reading chip")
	}
	time.Sleep(time.Millisecond)
	// Fuse AK8963 ROM access
	if mpu.i2cWrite(MPUREG_I2C_SLV0_DO, AK8963_I2CDIS); err != nil {
		return errors.New("ReadMagCalibration error reading chip")
	}
	time.Sleep(time.Millisecond)

	// Get sensitivity data from AK8963 fuse ROM
	mcal1, err := mpu.i2cRead(AK8963_ASAX)
	if err != nil {
		return errors.New("ReadMagCalibration error reading chip")
	}
	mcal2, err := mpu.i2cRead(AK8963_ASAY)
	if err != nil {
		return errors.New("ReadMagCalibration error reading chip")
	}
	mcal3, err := mpu.i2cRead(AK8963_ASAZ)
	if err != nil {
		return errors.New("ReadMagCalibration error reading chip")
	}

	mpu.mcal1 = float64(int16(mcal1)+128) / 256 * scaleMag
	mpu.mcal2 = float64(int16(mcal2)+128) / 256 * scaleMag
	mpu.mcal3 = float64(int16(mcal3)+128) / 256 * scaleMag

	// Clean up from getting sensitivity data from AK8963
	// Fuse AK8963 ROM access
	if err = mpu.i2cWrite(MPUREG_I2C_SLV0_DO, AK8963_I2CDIS); err != nil {
		return errors.New("ReadMagCalibration error reading chip")
	}
	time.Sleep(time.Millisecond)

	// Disable bypass mode now that we're done getting sensitivity data
	tmp, err = mpu.i2cRead(MPUREG_USER_CTRL)
	if err != nil {
		return errors.New("ReadMagCalibration error reading chip")
	}
	if err = mpu.i2cWrite(MPUREG_USER_CTRL, tmp|BIT_AUX_IF_EN); err != nil {
		return errors.New("ReadMagCalibration error reading chip")
	}
	time.Sleep(3 * time.Millisecond)
	if err = mpu.i2cWrite(MPUREG_INT_PIN_CFG, 0x00); err != nil {
		return errors.New("ReadMagCalibration error reading chip")
	}
	time.Sleep(3 * time.Millisecond)

	return nil
}

func (mpu *BMX160) i2cWrite(register, value byte) (err error) {

	if errWrite := mpu.i2cbus.WriteByteToReg(BMI160_I2C_ADDR, register, value); errWrite != nil {
		err = fmt.Errorf("BMX160 Error writing %X to %X: %s\n",
			value, register, errWrite.Error())
	} else {
		time.Sleep(time.Millisecond)
	}
	return
}

func (mpu *BMX160) i2cReadBytes(register byte, value []byte) (err error) {
	errRead := (mpu.i2cbus).ReadFromReg(BMI160_I2C_ADDR, register, value)
	if errRead != nil {
		err = fmt.Errorf("BMX160 Error reading from %X: %s", register, err)
	}
	return
}

func (mpu *BMX160) i2cRead(register byte) (value uint8, err error) {
	value, errWrite := mpu.i2cbus.ReadByteFromReg(BMI160_I2C_ADDR, register)
	if errWrite != nil {
		err = fmt.Errorf("i2cRead error: %s", errWrite.Error())
	}
	return
}

func (mpu *BMX160) i2cRead2(register byte) (value int16, err error) {

        buf := make([]byte, 2)
	if err := mpu.i2cbus.ReadFromReg(BMI160_I2C_ADDR, register, buf); err != nil {
		err = fmt.Errorf("BMX160 Error reading %x: %s\n", register, err.Error())
		return 0, err
	}

	value =  int16((uint16(buf[1]) << 8) | uint16(buf[0]))

	return
}

func (mpu *BMX160) memWrite(addr uint16, data *[]byte) error {
	var err error
	var tmp = make([]byte, 2)

	tmp[0] = byte(addr >> 8)
	tmp[1] = byte(addr & 0xFF)

	// Check memory bank boundaries
	if tmp[1]+byte(len(*data)) > MPU_BANK_SIZE {
		return errors.New("Bad address: writing outside of memory bank boundaries")
	}

	err = mpu.i2cbus.WriteToReg(BMI160_I2C_ADDR, MPUREG_BANK_SEL, tmp)
	if err != nil {
		return fmt.Errorf("BMX160 Error selecting memory bank: %s\n", err.Error())
	}

	err = mpu.i2cbus.WriteToReg(BMI160_I2C_ADDR, MPUREG_MEM_R_W, *data)
	if err != nil {
		return fmt.Errorf("BMX160 Error writing to the memory bank: %s\n", err.Error())
	}

	return nil
}
