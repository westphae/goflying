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
	bufSize  = 250 // Size of buffer storing instantaneous sensor values
	scaleMag = 9830.0 / 65536
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
	mcal1, mcal2, mcal3   float64   // Hardware magnetometer calibration values, uT
	cClose                chan bool // Turn off MPU polling
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
	icm.enableMag = enableMag

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

	// Enable the temperature DLPF. The chip's reset default is 0 (filter
	// bypassed, ~8 kHz BW), which makes every read full-bandwidth analog
	// noise — swings of 10–20 °C between consecutive samples. Setting
	// TEMP_DLPFCFG=4 gives ~34 Hz BW, which is plenty for a signal whose
	// real bandwidth is sub-Hz (chip thermal mass).
	if err := icm.setRegBank(2); err != nil {
		return nil, errors.New("Error selecting bank 2 for TEMP_CONFIG")
	}
	if err := icm.i2cWrite(ICMREG_TEMP_CONFIG, 0x04); err != nil {
		return nil, errors.New("Error configuring temperature DLPF")
	}
	if err := icm.setRegBank(0); err != nil {
		return nil, errors.New("Error restoring bank 0 after TEMP_CONFIG")
	}

	// Turn off FIFO buffer. Not necessary - default off.

	// Turn off interrupts. Not necessary - default off.

	// Set up magnetometer. The ICM-20948's onboard magnetometer is an AK09916,
	// not the AK8963 used in the MPU9250. Rather than configure the chip's
	// internal I²C master to relay AK09916 reads through EXT_SLV_SENS_DATA, we
	// enable bypass mode so the AK09916 appears directly at I²C 0x0C on the
	// host bus. readSensors then reads it via the embd I2CBus directly.
	if icm.enableMag {
		// Disable internal I²C master and route aux bus to host bus.
		if err := icm.i2cWrite(ICMREG_USER_CTRL, 0x00); err != nil {
			return nil, errors.New("Error disabling ICM-20948 I²C master")
		}
		if err := icm.i2cWrite(ICMREG_INT_PIN_CFG, BIT_BYPASS_EN); err != nil {
			return nil, errors.New("Error enabling ICM-20948 bypass mode")
		}
		time.Sleep(10 * time.Millisecond)

		wia, err := icm.i2cbus.ReadByteFromReg(AK09916_I2C_ADDR, AK09916_WIA2)
		if err != nil {
			return nil, fmt.Errorf("AK09916 not reachable at 0x%02X: %s", AK09916_I2C_ADDR, err.Error())
		}
		if wia != AK09916_DEVICE_ID {
			return nil, fmt.Errorf("AK09916 WIA2 mismatch: got 0x%02X want 0x%02X", wia, AK09916_DEVICE_ID)
		}

		// Pick target continuous-measurement rate. Modes 1 (10 Hz) and
		// 4 (100 Hz) silently produce zero data on at least some AK09916
		// dies; clamp the actual mag rate to 20 Hz (mode 2) or 50 Hz
		// (mode 3).
		var akMode byte = AK09916_CONTINUOUS_MODE2
		if icm.sampleRate >= 50 {
			akMode = AK09916_CONTINUOUS_MODE3
		}

		// Soft-reset the AK09916 (CNTL3=SRST). All AK09916 registers reset
		// to default and the chip drops into power-down regardless of any
		// prior state.
		if err := icm.i2cbus.WriteByteToReg(AK09916_I2C_ADDR, AK09916_CNTL3, AK09916_RESET); err != nil {
			return nil, errors.New("Error soft-resetting AK09916")
		}
		time.Sleep(100 * time.Millisecond)

		// On this die, the AK09916 silently ignores the first CNTL2 write
		// after a reset (whether the host ICM's PWR_MGMT_1 reset or our
		// own SRST), so write the target mode twice. The second write
		// always lands; the first either lands harmlessly or is absorbed.
		if err := icm.i2cbus.WriteByteToReg(AK09916_I2C_ADDR, AK09916_CNTL2, akMode); err != nil {
			return nil, errors.New("Error setting AK09916 continuous mode (1)")
		}
		time.Sleep(50 * time.Millisecond)
		if err := icm.i2cbus.WriteByteToReg(AK09916_I2C_ADDR, AK09916_CNTL2, akMode); err != nil {
			return nil, errors.New("Error setting AK09916 continuous mode (2)")
		}
		time.Sleep(100 * time.Millisecond)

		// Confirm the chip actually accepted the mode — definitive signal
		// if writes aren't landing at the AK09916 over bypass.
		if got, err := icm.i2cbus.ReadByteFromReg(AK09916_I2C_ADDR, AK09916_CNTL2); err != nil {
			log.Printf("ICM20948 warning: could not read back AK09916 CNTL2: %s", err.Error())
		} else if got != akMode {
			log.Printf("ICM20948 warning: AK09916 CNTL2 readback 0x%02X, expected 0x%02X — mag likely won't produce continuous data", got, akMode)
		}

		// AK09916 sensitivity is fixed at scaleMag µT/LSB; no factory ASA
		// registers like the AK8963 had.
		icm.mcal1 = scaleMag
		icm.mcal2 = scaleMag
		icm.mcal3 = scaleMag
	}
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
		g1, g2, g3, a1, a2, a3, m1, m2, m3, tmp   int16   // Current values
		avg1, avg2, avg3, ava1, ava2, ava3, avtmp float64 // Accumulators for averages
		avm1, avm2, avm3                          int32
		n, nm                                     float64
		gaError, magError                         error
		t0, t, t0m, tm                            time.Time
		magSampleRate                             int
		curdata                                   *sensors.IMUData
		akMissStreak                              int // consecutive DRDY=0 mag reads
		akMode                                    byte
	)
	akMode = AK09916_CONTINUOUS_MODE2
	if icm.sampleRate >= 50 {
		akMode = AK09916_CONTINUOUS_MODE3
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
		case t = <-clock.C: // Read accel/gyro/temp in one block:
			// Registers 0x2D..0x3A are contiguous (accel HL ×3, gyro HL ×3,
			// temp HL), big-endian. A single 14-byte block read is ~7×
			// fewer I²C transactions than reading each register pair,
			// and gives a torn-free snapshot of all six channels.
			var buf [14]byte
			if gaError = icm.i2cbus.ReadFromReg(icm.Address, ICMREG_ACCEL_XOUT_H, buf[:]); gaError != nil {
				log.Printf("ICM20948 Warning: error reading gyro/accel: %s", gaError.Error())
			} else {
				a1 = int16(uint16(buf[0])<<8 | uint16(buf[1]))
				a2 = int16(uint16(buf[2])<<8 | uint16(buf[3]))
				a3 = int16(uint16(buf[4])<<8 | uint16(buf[5]))
				g1 = int16(uint16(buf[6])<<8 | uint16(buf[7]))
				g2 = int16(uint16(buf[8])<<8 | uint16(buf[9]))
				g3 = int16(uint16(buf[10])<<8 | uint16(buf[11]))
				tmp = int16(uint16(buf[12])<<8 | uint16(buf[13]))
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
				// AK09916 requires reading ST1 (for DRDY) before the data
				// registers and ST2 after, in a single transaction, to
				// advance its state machine. Block-read 9 bytes from ST1:
				//   buf[0]    = ST1 (DRDY bit 0)
				//   buf[1..6] = HXL HXH HYL HYH HZL HZH (little-endian int16)
				//   buf[7]    = dummy (TMPS)
				//   buf[8]    = ST2 (HOFL bit 3); reading it releases latch.
				buf := make([]byte, 9)
				if magError = icm.i2cbus.ReadFromReg(AK09916_I2C_ADDR, AK09916_ST1, buf); magError != nil {
					log.Printf("ICM20948 Warning: error reading AK09916: %s", magError.Error())
					continue
				}
				if buf[0]&AKM_DATA_READY == 0 {
					// No new sample yet; leave m1/m2/m3 at previous values.
					// Transient I²C glitches on the gyro/accel side can
					// leave the AK09916 stuck in power-down with DRDY
					// permanently 0. After enough consecutive misses to
					// rule out normal timing, re-arm continuous mode.
					akMissStreak++
					if akMissStreak >= 8 {
						if err := icm.i2cbus.WriteByteToReg(AK09916_I2C_ADDR, AK09916_CNTL2, AK09916_POWER_DOWN); err == nil {
							icm.i2cbus.WriteByteToReg(AK09916_I2C_ADDR, AK09916_CNTL2, akMode)
						}
						akMissStreak = 0
					}
					continue
				}
				akMissStreak = 0
				if buf[8]&AK09916_HOFL != 0 {
					log.Println("ICM20948 mag data overflow")
					continue
				}
				m1 = int16(uint16(buf[1]) | uint16(buf[2])<<8)
				m2 = int16(uint16(buf[3]) | uint16(buf[4])<<8)
				m3 = int16(uint16(buf[5]) | uint16(buf[6])<<8)
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
// TODO westphae: need a way to start it going again!
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
		err = fmt.Errorf("ICM20948 Error reading %x: %s", register, errWrite.Error())
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
