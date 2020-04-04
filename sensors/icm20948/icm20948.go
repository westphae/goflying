package icm20948

// Approach adapted from the InvenSense DMP 6.1 drivers
// Also referenced https://github.com/brianc118/ICM20948/blob/master/ICM20948.cpp

import (
	"errors"
	"fmt"
	"log"
	"math"
	"time"

	"github.com/kidoman/embd"
	_ "github.com/kidoman/embd/host/all" // Empty import needed to initialize embd library.
	_ "github.com/kidoman/embd/host/rpi" // Empty import needed to initialize embd library.
	"github.com/westphae/goflying/sensors"
)

const (
	bufSize         = 250 // Size of buffer storing instantaneous sensor values
	scaleMag        = 9830.0 / 65536
)

/*
ICM20948 represents an InvenSense ICM20948 9DoF chip.
All communication is via channels.
*/
type ICM20948 struct {
	sensors.IMUSensor
	sensors.IMUCalData
	Address               byte
	i2cbus                embd.I2CBus
	scaleGyro, scaleAccel float64 // Max sensor reading for value 2**15-1
	sampleRate            int
	enableMag             bool
	mcal1, mcal2, mcal3   float64         // Hardware magnetometer calibration values, uT
	cClose                chan bool       // Turn off MPU polling
}

/*
NewICM20948 creates a new ICM20948 object according to the supplied parameters.  If there is no ICM20948 available or there
is an error creating the object, an error is returned.
*/
func NewICM20948(i2cbus *embd.I2CBus, address byte, sensitivityGyro, sensitivityAccel, sampleRate int, enableMag bool, applyHWOffsets bool) (*ICM20948, error) {
	var icm = new(ICM20948)
	if err := icm.IMUCalData.Load(); err != nil {
		icm.IMUCalData.Reset()
	}

	icm.sampleRate = sampleRate
	icm.enableMag = false //FIXME: enableMag. Always disabling magnetometer now.

	icm.i2cbus = *i2cbus
	icm.Address = address

	icm.setRegBank(0)

	// Initialization of MPU
	// Reset device.
	if err := icm.i2cWrite(ICMREG_PWR_MGMT_1, BIT_H_RESET); err != nil {
		return nil, errors.New("Error resetting ICM20948")
	}

	// Wake up chip.
	time.Sleep(100 * time.Millisecond)
	// CLKSEL = 1.
	// From ICM-20948 register map (PWR_MGMT_1):
	//  "NOTE: CLKSEL[2:0] should be set to 1~5 to achieve full gyroscope performance."
	if err := icm.i2cWrite(ICMREG_PWR_MGMT_1, 0x01); err != nil {
		return nil, errors.New("Error waking ICM20948")
	}

	// Note: inv_mpu.c sets some registers here to allocate 1kB to the FIFO buffer and 3kB to the DMP.
	// It doesn't seem to be supported in the 1.6 version of the register map and we're not using FIFO anyway,
	// so we skip this.
	// Don't let FIFO overwrite DMP data
	//if err := icm.i2cWrite(ICMREG_ACCEL_CONFIG_2, BIT_FIFO_SIZE_1024|0x8); err != nil {
	//	return nil, errors.New("Error setting up ICM20948")
	//}

	// Set Gyro and Accel sensitivities
	if err := icm.SetGyroSensitivity(sensitivityGyro); err != nil {
		log.Println(err)
	}

	if err := icm.SetAccelSensitivity(sensitivityAccel); err != nil {
		log.Println(err)
	}

	sampRate := byte(1125/icm.sampleRate - 1)
	// Default: Set Gyro LPF to half of sample rate
	if err := icm.SetGyroLPF(sampRate >> 1); err != nil {
		return nil, err
	}

	// Default: Set Accel LPF to half of sample rate
	if err := icm.SetAccelLPF(sampRate >> 1); err != nil {
		return nil, err
	}

	// Set sample rate to chosen
	if err := icm.SetGyroSampleRate(sampRate); err != nil {
		return nil, err
	}

	if err := icm.SetAccelSampleRate(sampRate); err != nil {
		return nil, err
	}

	// Turn off FIFO buffer. Not necessary - default off.

	// Turn off interrupts. Not necessary - default off.

	//FIXME. Mag reading not set up.
	// Set up magnetometer
	/*
		if icm.enableMag {
			if err := icm.ReadMagCalibration(); err != nil {
				return nil, errors.New("Error reading calibration from magnetometer")
			}

			// Set up AK8963 master mode, master clock and ES bit
			if err := icm.i2cWrite(ICMREG_I2C_MST_CTRL, 0x40); err != nil {
				return nil, errors.New("Error setting up AK8963")
			}
			// Slave 0 reads from AK8963
			if err := icm.i2cWrite(ICMREG_I2C_SLV0_ADDR, BIT_I2C_READ|AK8963_I2C_ADDR); err != nil {
				return nil, errors.New("Error setting up AK8963")
			}
			// Compass reads start at this register
			if err := icm.i2cWrite(ICMREG_I2C_SLV0_REG, AK8963_ST1); err != nil {
				return nil, errors.New("Error setting up AK8963")
			}
			// Enable 8-byte reads on slave 0
			if err := icm.i2cWrite(ICMREG_I2C_SLV0_CTRL, BIT_SLAVE_EN|8); err != nil {
				return nil, errors.New("Error setting up AK8963")
			}
			// Slave 1 can change AK8963 measurement mode
			if err := icm.i2cWrite(ICMREG_I2C_SLV1_ADDR, AK8963_I2C_ADDR); err != nil {
				return nil, errors.New("Error setting up AK8963")
			}
			if err := icm.i2cWrite(ICMREG_I2C_SLV1_REG, AK8963_CNTL1); err != nil {
				return nil, errors.New("Error setting up AK8963")
			}
			// Enable 1-byte reads on slave 1
			if err := icm.i2cWrite(ICMREG_I2C_SLV1_CTRL, BIT_SLAVE_EN|1); err != nil {
				return nil, errors.New("Error setting up AK8963")
			}
			// Set slave 1 data
			if err := icm.i2cWrite(ICMREG_I2C_SLV1_DO, AKM_SINGLE_MEASUREMENT); err != nil {
				return nil, errors.New("Error setting up AK8963")
			}
			// Triggers slave 0 and 1 actions at each sample
			if err := icm.i2cWrite(ICMREG_I2C_MST_DELAY_CTRL, 0x03); err != nil {
				return nil, errors.New("Error setting up AK8963")
			}

			// Set AK8963 sample rate to same as gyro/accel sample rate, up to max
			var ak8963Rate byte
			if icm.sampleRate < AK8963_MAX_SAMPLE_RATE {
				ak8963Rate = 0
			} else {
				ak8963Rate = byte(icm.sampleRate/AK8963_MAX_SAMPLE_RATE - 1)
			}

			// Not so sure of this one--I2C Slave 4??!
			if err := icm.i2cWrite(ICMREG_I2C_SLV4_CTRL, ak8963Rate); err != nil {
				return nil, errors.New("Error setting up AK8963")
			}

			time.Sleep(100 * time.Millisecond) // Make sure mag is ready
		}
	*/
	// Set clock source to PLL. Not necessary - default "auto select" (PLL when ready).

	if applyHWOffsets {
		if err := icm.ReadAccelBias(sensitivityAccel); err != nil {
			return nil, err
		}
		if err := icm.ReadGyroBias(sensitivityGyro); err != nil {
			return nil, err
		}
	}

	// Usually we don't want the automatic gyro bias compensation - it pollutes the gyro in a non-inertial frame.
	/*	if err := icm.EnableGyroBiasCal(false); err != nil {
			return nil, err
		}
	*/
	go icm.readSensors()

	// Give the IMU time to fully initialize and then clear out any bad values from the averages.
	time.Sleep(500 * time.Millisecond) // Make sure it's ready
	<-icm.CAvg                         // Discard the first readings.

	return icm, nil
}

// readSensors polls the gyro, accelerometer and magnetometer sensors as well as the die temperature.
// Communication is via channels.
func (icm *ICM20948) readSensors() {
	var (
		g1, g2, g3, a1, a2, a3, m1, m2, m3, m4, tmp int16   // Current values
		avg1, avg2, avg3, ava1, ava2, ava3, avtmp   float64 // Accumulators for averages
		avm1, avm2, avm3                            int32
		n, nm                                       float64
		gaError, magError                           error
		t0, t, t0m, tm                              time.Time
		magSampleRate                               int
		curdata                                     *sensors.IMUData
	)

	//FIXME: Temporary (testing).
	//	icm.setRegBank(2)
	//	icm.i2cWrite(ICMREG_TEMP_CONFIG, 0x04)
	//	icm.setRegBank(0)

	acRegMap := map[*int16]byte{
		&g1: ICMREG_GYRO_XOUT_H, &g2: ICMREG_GYRO_YOUT_H, &g3: ICMREG_GYRO_ZOUT_H,
		&a1: ICMREG_ACCEL_XOUT_H, &a2: ICMREG_ACCEL_YOUT_H, &a3: ICMREG_ACCEL_ZOUT_H,
		&tmp: ICMREG_TEMP_OUT_H,
	}
	magRegMap := map[*int16]byte{
		&m1: ICMREG_EXT_SENS_DATA_00, &m2: ICMREG_EXT_SENS_DATA_02, &m3: ICMREG_EXT_SENS_DATA_04, &m4: ICMREG_EXT_SENS_DATA_06,
	}

	if icm.sampleRate > 100 {
		magSampleRate = 100
	} else {
		magSampleRate = icm.sampleRate
	}

	cC := make(chan *sensors.IMUData)
	defer close(cC)
	icm.C = cC
	cAvg := make(chan *sensors.IMUData)
	defer close(cAvg)
	icm.CAvg = cAvg
	cBuf := make(chan *sensors.IMUData, bufSize)
	defer close(cBuf)
	icm.CBuf = cBuf
	icm.cClose = make(chan bool)
	defer close(icm.cClose)

	clock := time.NewTicker(time.Duration(int(1125.0/float32(icm.sampleRate)+0.5)) * time.Millisecond)
	//TODO westphae: use the clock to record actual time instead of a timer
	defer clock.Stop()

	clockMag := time.NewTicker(time.Duration(int(1125.0/float32(magSampleRate)+0.5)) * time.Millisecond)
	t0 = time.Now()
	t0m = time.Now()

	makeIMUData := func() *sensors.IMUData {
		mm1 := float64(m1)*icm.mcal1 - icm.M01
		mm2 := float64(m2)*icm.mcal2 - icm.M02
		mm3 := float64(m3)*icm.mcal3 - icm.M03
		//		fmt.Printf("a1=%d,a2=%d,a3=%d\n", a1, a2, a3)
		d := sensors.IMUData{
			G1:      (float64(g1) - icm.G01) * icm.scaleGyro,
			G2:      (float64(g2) - icm.G02) * icm.scaleGyro,
			G3:      (float64(g3) - icm.G03) * icm.scaleGyro,
			A1:      (float64(a1) - icm.A01) * icm.scaleAccel,
			A2:      (float64(a2) - icm.A02) * icm.scaleAccel,
			A3:      (float64(a3) - icm.A03) * icm.scaleAccel,
			M1:      icm.Ms11*mm1 + icm.Ms12*mm2 + icm.Ms13*mm3,
			M2:      icm.Ms21*mm1 + icm.Ms22*mm2 + icm.Ms23*mm3,
			M3:      icm.Ms31*mm1 + icm.Ms32*mm2 + icm.Ms33*mm3,
			Temp:    float64(tmp)/333.87 + 21.0,
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

	makeAvgIMUData := func() *sensors.IMUData {
		mm1 := float64(avm1)*icm.mcal1/nm - icm.M01
		mm2 := float64(avm2)*icm.mcal2/nm - icm.M02
		mm3 := float64(avm3)*icm.mcal3/nm - icm.M03
		d := sensors.IMUData{}
		if n > 0.5 {
			d.G1 = (avg1/n - icm.G01) * icm.scaleGyro
			d.G2 = (avg2/n - icm.G02) * icm.scaleGyro
			d.G3 = (avg3/n - icm.G03) * icm.scaleGyro
			d.A1 = (ava1/n - icm.A01) * icm.scaleAccel
			d.A2 = (ava2/n - icm.A02) * icm.scaleAccel
			d.A3 = (ava3/n - icm.A03) * icm.scaleAccel
			d.Temp = (float64(avtmp)/n)/333.87 + 21.0
			d.N = int(n + 0.5)
			d.T = t
			d.DT = t.Sub(t0)
		} else {
			d.GAError = errors.New("ICM20948 Error: No new accel/gyro values")
		}
		if nm > 0 {
			d.M1 = icm.Ms11*mm1 + icm.Ms12*mm2 + icm.Ms13*mm3
			d.M2 = icm.Ms21*mm1 + icm.Ms22*mm2 + icm.Ms23*mm3
			d.M3 = icm.Ms31*mm1 + icm.Ms32*mm2 + icm.Ms33*mm3
			d.NM = int(nm + 0.5)
			d.TM = tm
			d.DTM = t.Sub(t0m)
		} else {
			d.MagError = errors.New("ICM20948 Error: No new magnetometer values")
		}
		return &d
	}

	for {
		select {
		case t = <-clock.C: // Read accel/gyro data:
			for p, reg := range acRegMap {
				*p, gaError = icm.i2cRead2(reg)
				if gaError != nil {
					log.Println("ICM20948 Warning: error reading gyro/accel")
				}
			}
			curdata = makeIMUData()
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
			if icm.enableMag {
				// Set AK8963 to slave0 for reading
				if err := icm.i2cWrite(ICMREG_I2C_SLV0_ADDR, AK8963_I2C_ADDR|READ_FLAG); err != nil {
					log.Printf("ICM20948 Error: couldn't set AK8963 address for reading: %s", err.Error())
				}
				//I2C slave 0 register address from where to begin data transfer
				if err := icm.i2cWrite(ICMREG_I2C_SLV0_REG, AK8963_HXL); err != nil {
					log.Printf("ICM20948 Error: couldn't set AK8963 read register: %s", err.Error())
				}
				//Tell AK8963 that we will read 7 bytes
				if err := icm.i2cWrite(ICMREG_I2C_SLV0_CTRL, 0x87); err != nil {
					log.Printf("ICM20948 Error: couldn't communicate with AK8963: %s", err.Error())
				}

				// Read the actual data
				for p, reg := range magRegMap {
					*p, magError = icm.i2cRead2(reg)
					if magError != nil {
						log.Println("ICM20948 Warning: error reading magnetometer")
					}
				}

				// Test validity of magnetometer data
				if (byte(m1&0xFF)&AKM_DATA_READY) == 0x00 && (byte(m1&0xFF)&AKM_DATA_OVERRUN) != 0x00 {
					log.Println("ICM20948 mag data not ready or overflow")
					log.Printf("ICM20948 m1 LSB: %X\n", byte(m1&0xFF))
					continue // Don't update the accumulated values
				}

				if (byte((m4>>8)&0xFF) & AKM_OVERFLOW) != 0x00 {
					log.Println("ICM20948 mag data overflow")
					log.Printf("ICM20948 m4 MSB: %X\n", byte((m1>>8)&0xFF))
					continue // Don't update the accumulated values
				}

				// Update values and increment count of magnetometer readings
				avm1 += int32(m1)
				avm2 += int32(m2)
				avm3 += int32(m3)
				nm++
			}
		case cC <- curdata: // Send the latest values
		case cAvg <- makeAvgIMUData(): // Send the averages
			avg1, avg2, avg3 = 0, 0, 0
			ava1, ava2, ava3 = 0, 0, 0
			avm1, avm2, avm3 = 0, 0, 0
			avtmp = 0
			n, nm = 0, 0
			t0, t0m = t, tm
		case <-icm.cClose: // Stop the goroutine, ease up on the CPU
			break
		}
	}
}

// CloseMPU stops the driver from reading the MPU.
//TODO westphae: need a way to start it going again!
func (icm *ICM20948) CloseMPU() {
	// Nothing to do bitwise for the 9250?
	icm.cClose <- true
}

// SetGyroSampleRate changes the sampling rate of the gyro on the MPU.
func (icm *ICM20948) SetGyroSampleRate(rate byte) (err error) {
	// Gyro config registers on Bank 2.
	if errWrite := icm.setRegBank(2); errWrite != nil {
		return errors.New("ICM20948 Error: change register bank.")
	}

	defer icm.setRegBank(0)

	errWrite := icm.i2cWrite(ICMREG_GYRO_SMPLRT_DIV, byte(rate)) // Set sample rate to chosen
	if errWrite != nil {
		err = fmt.Errorf("ICM20948 Error: Couldn't set sample rate: %s", errWrite.Error())
	}
	return
}

// SetAccelSampleRate changes the sampling rate of the accelerometer on the MPU.
func (icm *ICM20948) SetAccelSampleRate(rate byte) (err error) {
	// Gyro config registers on Bank 2.
	if errWrite := icm.setRegBank(2); errWrite != nil {
		return errors.New("ICM20948 Error: change register bank.")
	}

	defer icm.setRegBank(0)

	errWrite := icm.i2cWrite(ICMREG_ACCEL_SMPLRT_DIV_2, byte(rate)) // Set sample rate to chosen
	if errWrite != nil {
		err = fmt.Errorf("ICM20948 Error: Couldn't set sample rate: %s", errWrite.Error())
	}
	return
}

// SetGyroLPF sets the low pass filter for the gyro.
func (icm *ICM20948) SetGyroLPF(rate byte) (err error) {
	var r byte

	// Gyro config registers on Bank 2.
	if errWrite := icm.setRegBank(2); errWrite != nil {
		return errors.New("ICM20948 Error: change register bank.")
	}

	defer icm.setRegBank(0)

	cfg, err := icm.i2cRead(ICMREG_GYRO_CONFIG)
	if err != nil {
		return errors.New("ICM20948 Error: SetGyroLPF error reading chip")
	}

	switch {
	case rate >= 197:
		r = BITS_DLPF_GYRO_CFG_197HZ
	case rate >= 152:
		r = BITS_DLPF_GYRO_CFG_152HZ
	case rate >= 120:
		r = BITS_DLPF_GYRO_CFG_120HZ
	case rate >= 51:
		r = BITS_DLPF_GYRO_CFG_51HZ
	case rate >= 24:
		r = BITS_DLPF_GYRO_CFG_24HZ
	case rate >= 12:
		r = BITS_DLPF_GYRO_CFG_12HZ
	default:
		r = BITS_DLPF_GYRO_CFG_6HZ
	}

	cfg |= 0x01
	cfg |= r

	errWrite := icm.i2cWrite(ICMREG_GYRO_CONFIG, cfg)
	if errWrite != nil {
		err = fmt.Errorf("ICM20948 Error: couldn't set Gyro LPF: %s", errWrite.Error())
	}
	return
}

// SetAccelLPF sets the low pass filter for the accelerometer.
func (icm *ICM20948) SetAccelLPF(rate byte) (err error) {
	var r byte

	// Accel config registers on Bank 2.
	if errWrite := icm.setRegBank(2); errWrite != nil {
		return errors.New("ICM20948 Error: change register bank.")
	}

	defer icm.setRegBank(0)

	cfg, err := icm.i2cRead(ICMREG_ACCEL_CONFIG)
	if err != nil {
		return errors.New("ICM20948 Error: SetGyroLPF error reading chip")
	}

	switch {
	case rate >= 246:
		r = BITS_DLPF_ACCEL_CFG_246HZ
	case rate >= 111:
		r = BITS_DLPF_ACCEL_CFG_111HZ
	case rate >= 50:
		r = BITS_DLPF_ACCEL_CFG_50HZ
	case rate >= 24:
		r = BITS_DLPF_ACCEL_CFG_24HZ
	case rate >= 12:
		r = BITS_DLPF_ACCEL_CFG_12HZ
	default:
		r = BITS_DLPF_ACCEL_CFG_5HZ
	}

	cfg |= 0x01
	cfg |= r

	errWrite := icm.i2cWrite(ICMREG_ACCEL_CONFIG, cfg)
	if errWrite != nil {
		err = fmt.Errorf("ICM20948 Error: couldn't set Accel LPF: %s", errWrite.Error())
	}
	return
}

// EnableGyroBiasCal enables or disables motion bias compensation for the gyro.
// For flying we generally do not want this!
func (icm *ICM20948) EnableGyroBiasCal(enable bool) error {
	enableRegs := []byte{0xb8, 0xaa, 0xb3, 0x8d, 0xb4, 0x98, 0x0d, 0x35, 0x5d}
	disableRegs := []byte{0xb8, 0xaa, 0xaa, 0xaa, 0xb0, 0x88, 0xc3, 0xc5, 0xc7}

	if enable {
		if err := icm.memWrite(CFG_MOTION_BIAS, &enableRegs); err != nil {
			return errors.New("Unable to enable motion bias compensation")
		}
	} else {
		if err := icm.memWrite(CFG_MOTION_BIAS, &disableRegs); err != nil {
			return errors.New("Unable to disable motion bias compensation")
		}
	}

	return nil
}

// SampleRate returns the current sample rate of the ICM20948, in Hz.
func (icm *ICM20948) SampleRate() int {
	return icm.sampleRate
}

// MagEnabled returns whether or not the magnetometer is being read.
func (icm *ICM20948) MagEnabled() bool {
	return icm.enableMag
}

// SetGyroSensitivity sets the gyro sensitivity of the ICM20948; it must be one of the following values:
// 250, 500, 1000, 2000 (all in deg/s).
func (icm *ICM20948) SetGyroSensitivity(sensitivityGyro int) (err error) {
	var sensGyro byte

	// Gyro config registers on Bank 2.
	if errWrite := icm.setRegBank(2); errWrite != nil {
		return errors.New("ICM20948 Error: change register bank.")
	}

	defer icm.setRegBank(0)

	switch sensitivityGyro {
	case 2000:
		sensGyro = BITS_FS_2000DPS
		icm.scaleGyro = 2000.0 / float64(math.MaxInt16)
	case 1000:
		sensGyro = BITS_FS_1000DPS
		icm.scaleGyro = 1000.0 / float64(math.MaxInt16)
	case 500:
		sensGyro = BITS_FS_500DPS
		icm.scaleGyro = 500.0 / float64(math.MaxInt16)
	case 250:
		sensGyro = BITS_FS_250DPS
		icm.scaleGyro = 250.0 / float64(math.MaxInt16)
	default:
		err = fmt.Errorf("ICM20948 Error: %d is not a valid gyro sensitivity", sensitivityGyro)
	}

	if errWrite := icm.i2cWrite(ICMREG_GYRO_CONFIG, sensGyro); errWrite != nil {
		err = errors.New("ICM20948 Error: couldn't set gyro sensitivity")
	}

	return
}

func (icm *ICM20948) setRegBank(bank byte) error {
	return icm.i2cWrite(ICMREG_BANK_SEL, bank<<4)
}

// SetAccelSensitivity sets the accelerometer sensitivity of the ICM20948; it must be one of the following values:
// 2, 4, 8, 16, all in G (gravity).
func (icm *ICM20948) SetAccelSensitivity(sensitivityAccel int) error {
	var sensAccel byte

	// Accel config registers on Bank 2.
	if errWrite := icm.setRegBank(2); errWrite != nil {
		return errors.New("ICM20948 Error: change register bank.")
	}

	defer icm.setRegBank(0)

	switch sensitivityAccel {
	case 16:
		sensAccel = BITS_FS_16G
		icm.scaleAccel = 16.0 / float64(math.MaxInt16)
	case 8:
		sensAccel = BITS_FS_8G
		icm.scaleAccel = 8.0 / float64(math.MaxInt16)
	case 4:
		sensAccel = BITS_FS_4G
		icm.scaleAccel = 4.0 / float64(math.MaxInt16)
	case 2:
		sensAccel = BITS_FS_2G
		icm.scaleAccel = 2.0 / float64(math.MaxInt16)
	default:
		return fmt.Errorf("ICM20948 Error: %d is not a valid accel sensitivity", sensitivityAccel)
	}

	if errWrite := icm.i2cWrite(ICMREG_ACCEL_CONFIG, sensAccel); errWrite != nil {
		return errors.New("ICM20948 Error: couldn't set accel sensitivity")
	}

	return nil
}

// ReadAccelBias reads the bias accelerometer value stored on the chip.
// These values are set at the factory.
func (icm *ICM20948) ReadAccelBias(sensitivityAccel int) error {
	if errWrite := icm.setRegBank(1); errWrite != nil {
		return errors.New("ICM20948 Error: change register bank.")
	}
	defer icm.setRegBank(0)

	a0x, err := icm.i2cRead2(ICMREG_XA_OFFSET_H)
	if err != nil {
		return errors.New("ICM20948 Error: ReadAccelBias error reading chip")
	}
	a0y, err := icm.i2cRead2(ICMREG_YA_OFFSET_H)
	if err != nil {
		return errors.New("ICM20948 Error: ReadAccelBias error reading chip")
	}
	a0z, err := icm.i2cRead2(ICMREG_ZA_OFFSET_H)
	if err != nil {
		return errors.New("ICM20948 Error: ReadAccelBias error reading chip")
	}

	switch sensitivityAccel {
	case 16:
		icm.A01 = float64(a0x >> 1)
		icm.A02 = float64(a0y >> 1)
		icm.A03 = float64(a0z >> 1)
	case 8:
		icm.A01 = float64(a0x)
		icm.A02 = float64(a0y)
		icm.A03 = float64(a0z)
	case 4:
		icm.A01 = float64(a0x << 1)
		icm.A02 = float64(a0y << 1)
		icm.A03 = float64(a0z << 1)
	case 2:
		icm.A01 = float64(a0x << 2)
		icm.A02 = float64(a0y << 2)
		icm.A03 = float64(a0z << 2)
	default:
		return fmt.Errorf("ICM20948 Error: %d is not a valid acceleration sensitivity", sensitivityAccel)
	}

	return nil
}

// ReadGyroBias reads the bias gyro value stored on the chip.
// These values are set at the factory.
func (icm *ICM20948) ReadGyroBias(sensitivityGyro int) error {
	if errWrite := icm.setRegBank(2); errWrite != nil {
		return errors.New("ICM20948 Error: change register bank.")
	}
	defer icm.setRegBank(0)

	g0x, err := icm.i2cRead2(ICMREG_XG_OFFS_USRH)
	if err != nil {
		return errors.New("ICM20948 Error: ReadGyroBias error reading chip")
	}
	g0y, err := icm.i2cRead2(ICMREG_YG_OFFS_USRH)
	if err != nil {
		return errors.New("ICM20948 Error: ReadGyroBias error reading chip")
	}
	g0z, err := icm.i2cRead2(ICMREG_ZG_OFFS_USRH)
	if err != nil {
		return errors.New("ICM20948 Error: ReadGyroBias error reading chip")
	}

	switch sensitivityGyro {
	case 2000:
		icm.G01 = float64(g0x >> 1)
		icm.G02 = float64(g0y >> 1)
		icm.G03 = float64(g0z >> 1)
	case 1000:
		icm.G01 = float64(g0x)
		icm.G02 = float64(g0y)
		icm.G03 = float64(g0z)
	case 500:
		icm.G01 = float64(g0x << 1)
		icm.G02 = float64(g0y << 1)
		icm.G03 = float64(g0z << 1)
	case 250:
		icm.G01 = float64(g0x << 2)
		icm.G02 = float64(g0y << 2)
		icm.G03 = float64(g0z << 2)
	default:
		return fmt.Errorf("ICM20948 Error: %d is not a valid gyro sensitivity", sensitivityGyro)
	}

	return nil
}

// ReadMagCalibration reads the magnetometer bias values stored on the chpi.
// These values are set at the factory.
func (icm *ICM20948) ReadMagCalibration() error {
	// Enable bypass mode
	var tmp uint8
	var err error
	tmp, err = icm.i2cRead(ICMREG_USER_CTRL)
	if err != nil {
		return errors.New("ReadMagCalibration error reading chip")
	}
	if err = icm.i2cWrite(ICMREG_USER_CTRL, tmp & ^BIT_AUX_IF_EN); err != nil {
		return errors.New("ReadMagCalibration error reading chip")
	}
	time.Sleep(3 * time.Millisecond)
	if err = icm.i2cWrite(ICMREG_INT_PIN_CFG, BIT_BYPASS_EN); err != nil {
		return errors.New("ReadMagCalibration error reading chip")
	}

	// Prepare for getting sensitivity data from AK8963
	//Set the I2C slave address of AK8963
	if err = icm.i2cWrite(ICMREG_I2C_SLV0_ADDR, AK8963_I2C_ADDR); err != nil {
		return errors.New("ReadMagCalibration error reading chip")
	}
	// Power down the AK8963
	if err = icm.i2cWrite(ICMREG_I2C_SLV0_CTRL, AK8963_CNTL1); err != nil {
		return errors.New("ReadMagCalibration error reading chip")
	}
	// Power down the AK8963
	if err = icm.i2cWrite(ICMREG_I2C_SLV0_DO, AKM_POWER_DOWN); err != nil {
		return errors.New("ReadMagCalibration error reading chip")
	}
	time.Sleep(time.Millisecond)
	// Fuse AK8963 ROM access
	if icm.i2cWrite(ICMREG_I2C_SLV0_DO, AK8963_I2CDIS); err != nil {
		return errors.New("ReadMagCalibration error reading chip")
	}
	time.Sleep(time.Millisecond)

	// Get sensitivity data from AK8963 fuse ROM
	mcal1, err := icm.i2cRead(AK8963_ASAX)
	if err != nil {
		return errors.New("ReadMagCalibration error reading chip")
	}
	mcal2, err := icm.i2cRead(AK8963_ASAY)
	if err != nil {
		return errors.New("ReadMagCalibration error reading chip")
	}
	mcal3, err := icm.i2cRead(AK8963_ASAZ)
	if err != nil {
		return errors.New("ReadMagCalibration error reading chip")
	}

	icm.mcal1 = float64(int16(mcal1)+128) / 256 * scaleMag
	icm.mcal2 = float64(int16(mcal2)+128) / 256 * scaleMag
	icm.mcal3 = float64(int16(mcal3)+128) / 256 * scaleMag

	// Clean up from getting sensitivity data from AK8963
	// Fuse AK8963 ROM access
	if err = icm.i2cWrite(ICMREG_I2C_SLV0_DO, AK8963_I2CDIS); err != nil {
		return errors.New("ReadMagCalibration error reading chip")
	}
	time.Sleep(time.Millisecond)

	// Disable bypass mode now that we're done getting sensitivity data
	tmp, err = icm.i2cRead(ICMREG_USER_CTRL)
	if err != nil {
		return errors.New("ReadMagCalibration error reading chip")
	}
	if err = icm.i2cWrite(ICMREG_USER_CTRL, tmp|BIT_AUX_IF_EN); err != nil {
		return errors.New("ReadMagCalibration error reading chip")
	}
	time.Sleep(3 * time.Millisecond)
	if err = icm.i2cWrite(ICMREG_INT_PIN_CFG, 0x00); err != nil {
		return errors.New("ReadMagCalibration error reading chip")
	}
	time.Sleep(3 * time.Millisecond)

	return nil
}

func (icm *ICM20948) i2cWrite(register, value byte) (err error) {

	if errWrite := icm.i2cbus.WriteByteToReg(icm.Address, register, value); errWrite != nil {
		err = fmt.Errorf("ICM20948 Error writing %X to %X: %s\n",
			value, register, errWrite.Error())
	} else {
		time.Sleep(time.Millisecond)
	}
	return
}

func (icm *ICM20948) i2cRead(register byte) (value uint8, err error) {
	value, errWrite := icm.i2cbus.ReadByteFromReg(icm.Address, register)
	if errWrite != nil {
		err = fmt.Errorf("i2cRead error: %s", errWrite.Error())
	}
	return
}

func (icm *ICM20948) i2cRead2(register byte) (value int16, err error) {

	v, errWrite := icm.i2cbus.ReadWordFromReg(icm.Address, register)
	if errWrite != nil {
		err = fmt.Errorf("ICM20948 Error reading %x: %s\n", register, err.Error())
	} else {
		value = int16(v)
	}
	return
}

func (icm *ICM20948) memWrite(addr uint16, data *[]byte) error {
	var err error
	var tmp = make([]byte, 2)

	tmp[0] = byte(addr >> 8)
	tmp[1] = byte(addr & 0xFF)

	// Check memory bank boundaries
	if tmp[1]+byte(len(*data)) > MPU_BANK_SIZE {
		return errors.New("Bad address: writing outside of memory bank boundaries")
	}

	err = icm.i2cbus.WriteToReg(icm.Address, ICMREG_BANK_SEL, tmp)
	if err != nil {
		return fmt.Errorf("ICM20948 Error selecting memory bank: %s\n", err.Error())
	}

	err = icm.i2cbus.WriteToReg(icm.Address, ICMREG_MEM_R_W, *data)
	if err != nil {
		return fmt.Errorf("ICM20948 Error writing to the memory bank: %s\n", err.Error())
	}

	return nil
}
