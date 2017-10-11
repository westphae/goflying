package mpu9250

// Approach adapted from the InvenSense DMP 6.1 drivers
// Also referenced https://github.com/brianc118/MPU9250/blob/master/MPU9250.cpp

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
	bufSize         = 250 // Size of buffer storing instantaneous sensor values
	scaleMag        = 9830.0 / 65536
	calDataLocation = "/etc/mpu9250cal.json"
)

// MPUData contains all the values measured by an MPU9250.
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
		log.Printf("MPU9250: Error saving calibration data to %s: %s", calDataLocation, err.Error())
		return
	}
	defer fd.Close()
	calData, err := json.Marshal(d)
	if err != nil {
		log.Printf("MPU9250: Error marshaling calibration data: %s", err)
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
	errstr := "MPU9250: Error reading calibration data from %s: %s"
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
MPU9250 represents an InvenSense MPU9250 9DoF chip.
All communication is via channels.
*/
type MPU9250 struct {
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
NewMPU9250 creates a new MPU9250 object according to the supplied parameters.  If there is no MPU9250 available or there
is an error creating the object, an error is returned.
*/
func NewMPU9250(sensitivityGyro, sensitivityAccel, sampleRate int, enableMag bool, applyHWOffsets bool) (*MPU9250, error) {
	var mpu = new(MPU9250)
	if err := mpu.mpuCalData.load(); err != nil {
		mpu.mpuCalData.reset()
	}

	mpu.sampleRate = sampleRate
	mpu.enableMag = enableMag

	mpu.i2cbus = embd.NewI2CBus(1)

	// Initialization of MPU
	// Reset device.
	if err := mpu.i2cWrite(MPUREG_PWR_MGMT_1, BIT_H_RESET); err != nil {
		return nil, errors.New("Error resetting MPU9250")
	}

	// Note: the following is in inv_mpu.c, but doesn't appear to be necessary from the MPU-9250 register map.
	// Wake up chip.
	time.Sleep(100 * time.Millisecond)
	if err := mpu.i2cWrite(MPUREG_PWR_MGMT_1, 0x00); err != nil {
		return nil, errors.New("Error waking MPU9250")
	}

	// Note: inv_mpu.c sets some registers here to allocate 1kB to the FIFO buffer and 3kB to the DMP.
	// It doesn't seem to be supported in the 1.6 version of the register map and we're not using FIFO anyway,
	// so we skip this.
	// Don't let FIFO overwrite DMP data
	if err := mpu.i2cWrite(MPUREG_ACCEL_CONFIG_2, BIT_FIFO_SIZE_1024|0x8); err != nil {
		return nil, errors.New("Error setting up MPU9250")
	}

	// Set Gyro and Accel sensitivities
	if err := mpu.SetGyroSensitivity(sensitivityGyro); err != nil {
		log.Println(err)
	}

	if err := mpu.SetAccelSensitivity(sensitivityAccel); err != nil {
		log.Println(err)
	}

	sampRate := byte(1000/mpu.sampleRate - 1)
	// Default: Set Gyro LPF to half of sample rate
	if err := mpu.SetGyroLPF(sampRate >> 1); err != nil {
		return nil, err
	}

	// Default: Set Accel LPF to half of sample rate
	if err := mpu.SetAccelLPF(sampRate >> 1); err != nil {
		return nil, err
	}

	// Set sample rate to chosen
	if err := mpu.SetSampleRate(sampRate); err != nil {
		return nil, err
	}

	// Turn off FIFO buffer
	if err := mpu.i2cWrite(MPUREG_FIFO_EN, 0x00); err != nil {
		return nil, errors.New("MPU9250 Error: couldn't disable FIFO")
	}

	// Turn off interrupts
	if err := mpu.i2cWrite(MPUREG_INT_ENABLE, 0x00); err != nil {
		return nil, errors.New("MPU9250 Error: couldn't disable interrupts")
	}

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

	// Set clock source to PLL
	if err := mpu.i2cWrite(MPUREG_PWR_MGMT_1, INV_CLK_PLL); err != nil {
		return nil, errors.New("Error setting up MPU9250")
	}
	// Turn off all sensors -- Not sure if necessary, but it's in the InvenSense DMP driver
	if err := mpu.i2cWrite(MPUREG_PWR_MGMT_2, 0x63); err != nil {
		return nil, errors.New("Error setting up MPU9250")
	}
	time.Sleep(100 * time.Millisecond)
	// Turn on all gyro, all accel
	if err := mpu.i2cWrite(MPUREG_PWR_MGMT_2, 0x00); err != nil {
		return nil, errors.New("Error setting up MPU9250")
	}

	if applyHWOffsets {
		if err := mpu.ReadAccelBias(sensitivityAccel); err != nil {
			return nil, err
		}
		if err := mpu.ReadGyroBias(sensitivityGyro); err != nil {
			return nil, err
		}
	}

	// Usually we don't want the automatic gyro bias compensation - it pollutes the gyro in a non-inertial frame.
	if err := mpu.EnableGyroBiasCal(false); err != nil {
		return nil, err
	}

	go mpu.readSensors()

	// Give the IMU time to fully initialize and then clear out any bad values from the averages.
	time.Sleep(500 * time.Millisecond) // Make sure it's ready
	<-mpu.CAvg

	return mpu, nil
}

// readSensors polls the gyro, accelerometer and magnetometer sensors as well as the die temperature.
// Communication is via channels.
func (mpu *MPU9250) readSensors() {
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
		&g1: MPUREG_GYRO_XOUT_H, &g2: MPUREG_GYRO_YOUT_H, &g3: MPUREG_GYRO_ZOUT_H,
		&a1: MPUREG_ACCEL_XOUT_H, &a2: MPUREG_ACCEL_YOUT_H, &a3: MPUREG_ACCEL_ZOUT_H,
		&tmp: MPUREG_TEMP_OUT_H,
	}
	magRegMap := map[*int16]byte{
		&m1: MPUREG_EXT_SENS_DATA_00, &m2: MPUREG_EXT_SENS_DATA_02, &m3: MPUREG_EXT_SENS_DATA_04, &m4: MPUREG_EXT_SENS_DATA_06,
	}

	if mpu.sampleRate > 100 {
		magSampleRate = 100
	} else {
		magSampleRate = mpu.sampleRate
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
			G1:      (float64(g1) - mpu.G01) * mpu.scaleGyro,
			G2:      (float64(g2) - mpu.G02) * mpu.scaleGyro,
			G3:      (float64(g3) - mpu.G03) * mpu.scaleGyro,
			A1:      (float64(a1) - mpu.A01) * mpu.scaleAccel,
			A2:      (float64(a2) - mpu.A02) * mpu.scaleAccel,
			A3:      (float64(a3) - mpu.A03) * mpu.scaleAccel,
			M1:      mpu.Ms11*mm1 + mpu.Ms12*mm2 + mpu.Ms13*mm3,
			M2:      mpu.Ms21*mm1 + mpu.Ms22*mm2 + mpu.Ms23*mm3,
			M3:      mpu.Ms31*mm1 + mpu.Ms32*mm2 + mpu.Ms33*mm3,
			Temp:    float64(tmp)/340 + 36.53,
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
			d.G1 = (avg1/n - mpu.G01) * mpu.scaleGyro
			d.G2 = (avg2/n - mpu.G02) * mpu.scaleGyro
			d.G3 = (avg3/n - mpu.G03) * mpu.scaleGyro
			d.A1 = (ava1/n - mpu.A01) * mpu.scaleAccel
			d.A2 = (ava2/n - mpu.A02) * mpu.scaleAccel
			d.A3 = (ava3/n - mpu.A03) * mpu.scaleAccel
			d.Temp = (float64(avtmp)/n)/340 + 36.53
			d.N = int(n + 0.5)
			d.T = t
			d.DT = t.Sub(t0)
		} else {
			d.GAError = errors.New("MPU9250 Error: No new accel/gyro values")
		}
		if nm > 0 {
			d.M1 = mpu.Ms11*mm1 + mpu.Ms12*mm2 + mpu.Ms13*mm3
			d.M2 = mpu.Ms21*mm1 + mpu.Ms22*mm2 + mpu.Ms23*mm3
			d.M3 = mpu.Ms31*mm1 + mpu.Ms32*mm2 + mpu.Ms33*mm3
			d.NM = int(nm + 0.5)
			d.TM = tm
			d.DTM = t.Sub(t0m)
		} else {
			d.MagError = errors.New("MPU9250 Error: No new magnetometer values")
		}
		return &d
	}

	for {
		select {
		case t = <-clock.C: // Read accel/gyro data:
			for p, reg := range acRegMap {
				*p, gaError = mpu.i2cRead2(reg)
				if gaError != nil {
					log.Println("MPU9250 Warning: error reading gyro/accel")
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
func (mpu *MPU9250) CloseMPU() {
	// Nothing to do bitwise for the 9250?
	mpu.cClose <- true
}

// SetSampleRate changes the sampling rate of the MPU.
func (mpu *MPU9250) SetSampleRate(rate byte) (err error) {
	errWrite := mpu.i2cWrite(MPUREG_SMPLRT_DIV, byte(rate)) // Set sample rate to chosen
	if errWrite != nil {
		err = fmt.Errorf("MPU9250 Error: Couldn't set sample rate: %s", errWrite.Error())
	}
	return
}

// SetGyroLPF sets the low pass filter for the gyro.
func (mpu *MPU9250) SetGyroLPF(rate byte) (err error) {
	var r byte
	switch {
	case rate >= 188:
		r = BITS_DLPF_CFG_188HZ
	case rate >= 98:
		r = BITS_DLPF_CFG_98HZ
	case rate >= 42:
		r = BITS_DLPF_CFG_42HZ
	case rate >= 20:
		r = BITS_DLPF_CFG_20HZ
	case rate >= 10:
		r = BITS_DLPF_CFG_10HZ
	default:
		r = BITS_DLPF_CFG_5HZ
	}

	errWrite := mpu.i2cWrite(MPUREG_CONFIG, r)
	if errWrite != nil {
		err = fmt.Errorf("MPU9250 Error: couldn't set Gyro LPF: %s", errWrite.Error())
	}
	return
}

// SetAccelLPF sets the low pass filter for the accelerometer.
func (mpu *MPU9250) SetAccelLPF(rate byte) (err error) {
	var r byte
	switch {
	case rate >= 218:
		r = BITS_DLPF_CFG_188HZ
	case rate >= 99:
		r = BITS_DLPF_CFG_98HZ
	case rate >= 45:
		r = BITS_DLPF_CFG_42HZ
	case rate >= 21:
		r = BITS_DLPF_CFG_20HZ
	case rate >= 10:
		r = BITS_DLPF_CFG_10HZ
	default:
		r = BITS_DLPF_CFG_5HZ
	}

	errWrite := mpu.i2cWrite(MPUREG_ACCEL_CONFIG_2, r)
	if errWrite != nil {
		err = fmt.Errorf("MPU9250 Error: couldn't set Accel LPF: %s", errWrite.Error())
	}
	return
}

// EnableGyroBiasCal enables or disables motion bias compensation for the gyro.
// For flying we generally do not want this!
func (mpu *MPU9250) EnableGyroBiasCal(enable bool) error {
	enableRegs := []byte{0xb8, 0xaa, 0xb3, 0x8d, 0xb4, 0x98, 0x0d, 0x35, 0x5d}
	disableRegs := []byte{0xb8, 0xaa, 0xaa, 0xaa, 0xb0, 0x88, 0xc3, 0xc5, 0xc7}

	if enable {
		if err := mpu.memWrite(CFG_MOTION_BIAS, &enableRegs); err != nil {
			return errors.New("Unable to enable motion bias compensation")
		}
	} else {
		if err := mpu.memWrite(CFG_MOTION_BIAS, &disableRegs); err != nil {
			return errors.New("Unable to disable motion bias compensation")
		}
	}

	return nil
}

// SampleRate returns the current sample rate of the MPU9250, in Hz.
func (mpu *MPU9250) SampleRate() int {
	return mpu.sampleRate
}

// MagEnabled returns whether or not the magnetometer is being read.
func (mpu *MPU9250) MagEnabled() bool {
	return mpu.enableMag
}

// SetGyroSensitivity sets the gyro sensitivity of the MPU9250; it must be one of the following values:
// 250, 500, 1000, 2000 (all in °/s).
func (mpu *MPU9250) SetGyroSensitivity(sensitivityGyro int) (err error) {
	var sensGyro byte

	switch sensitivityGyro {
	case 2000:
		sensGyro = BITS_FS_2000DPS
		mpu.scaleGyro = 2000.0 / float64(math.MaxInt16)
	case 1000:
		sensGyro = BITS_FS_1000DPS
		mpu.scaleGyro = 1000.0 / float64(math.MaxInt16)
	case 500:
		sensGyro = BITS_FS_500DPS
		mpu.scaleGyro = 500.0 / float64(math.MaxInt16)
	case 250:
		sensGyro = BITS_FS_250DPS
		mpu.scaleGyro = 250.0 / float64(math.MaxInt16)
	default:
		err = fmt.Errorf("MPU9250 Error: %d is not a valid gyro sensitivity", sensitivityGyro)
	}

	if errWrite := mpu.i2cWrite(MPUREG_GYRO_CONFIG, sensGyro); errWrite != nil {
		err = errors.New("MPU9250 Error: couldn't set gyro sensitivity")
	}

	return
}

// SetAccelSensitivity sets the accelerometer sensitivity of the MPU9250; it must be one of the following values:
// 2, 4, 8, 16, all in G (gravity).
func (mpu *MPU9250) SetAccelSensitivity(sensitivityAccel int) (err error) {
	var sensAccel byte

	switch sensitivityAccel {
	case 16:
		sensAccel = BITS_FS_16G
		mpu.scaleAccel = 16.0 / float64(math.MaxInt16)
	case 8:
		sensAccel = BITS_FS_8G
		mpu.scaleAccel = 8.0 / float64(math.MaxInt16)
	case 4:
		sensAccel = BITS_FS_4G
		mpu.scaleAccel = 4.0 / float64(math.MaxInt16)
	case 2:
		sensAccel = BITS_FS_2G
		mpu.scaleAccel = 2.0 / float64(math.MaxInt16)
	default:
		err = fmt.Errorf("MPU9250 Error: %d is not a valid accel sensitivity", sensitivityAccel)
	}

	if errWrite := mpu.i2cWrite(MPUREG_ACCEL_CONFIG, sensAccel); errWrite != nil {
		err = errors.New("MPU9250 Error: couldn't set accel sensitivity")
	}

	return
}

// ReadAccelBias reads the bias accelerometer value stored on the chip.
// These values are set at the factory.
func (mpu *MPU9250) ReadAccelBias(sensitivityAccel int) error {
	a0x, err := mpu.i2cRead2(MPUREG_XA_OFFSET_H)
	if err != nil {
		return errors.New("MPU9250 Error: ReadAccelBias error reading chip")
	}
	a0y, err := mpu.i2cRead2(MPUREG_YA_OFFSET_H)
	if err != nil {
		return errors.New("MPU9250 Error: ReadAccelBias error reading chip")
	}
	a0z, err := mpu.i2cRead2(MPUREG_ZA_OFFSET_H)
	if err != nil {
		return errors.New("MPU9250 Error: ReadAccelBias error reading chip")
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
		return fmt.Errorf("MPU9250 Error: %d is not a valid acceleration sensitivity", sensitivityAccel)
	}

	return nil
}

// ReadGyroBias reads the bias gyro value stored on the chip.
// These values are set at the factory.
func (mpu *MPU9250) ReadGyroBias(sensitivityGyro int) error {
	g0x, err := mpu.i2cRead2(MPUREG_XG_OFFS_USRH)
	if err != nil {
		return errors.New("MPU9250 Error: ReadGyroBias error reading chip")
	}
	g0y, err := mpu.i2cRead2(MPUREG_YG_OFFS_USRH)
	if err != nil {
		return errors.New("MPU9250 Error: ReadGyroBias error reading chip")
	}
	g0z, err := mpu.i2cRead2(MPUREG_ZG_OFFS_USRH)
	if err != nil {
		return errors.New("MPU9250 Error: ReadGyroBias error reading chip")
	}

	switch sensitivityGyro {
	case 2000:
		mpu.G01 = float64(g0x >> 1)
		mpu.G02 = float64(g0y >> 1)
		mpu.G03 = float64(g0z >> 1)
	case 1000:
		mpu.G01 = float64(g0x)
		mpu.G02 = float64(g0y)
		mpu.G03 = float64(g0z)
	case 500:
		mpu.G01 = float64(g0x << 1)
		mpu.G02 = float64(g0y << 1)
		mpu.G03 = float64(g0z << 1)
	case 250:
		mpu.G01 = float64(g0x << 2)
		mpu.G02 = float64(g0y << 2)
		mpu.G03 = float64(g0z << 2)
	default:
		return fmt.Errorf("MPU9250 Error: %d is not a valid gyro sensitivity", sensitivityGyro)
	}

	return nil
}

// ReadMagCalibration reads the magnetometer bias values stored on the chpi.
// These values are set at the factory.
func (mpu *MPU9250) ReadMagCalibration() error {
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

func (mpu *MPU9250) i2cWrite(register, value byte) (err error) {

	if errWrite := mpu.i2cbus.WriteByteToReg(MPU_ADDRESS, register, value); errWrite != nil {
		err = fmt.Errorf("MPU9250 Error writing %X to %X: %s\n",
			value, register, errWrite.Error())
	} else {
		time.Sleep(time.Millisecond)
	}
	return
}

func (mpu *MPU9250) i2cRead(register byte) (value uint8, err error) {
	value, errWrite := mpu.i2cbus.ReadByteFromReg(MPU_ADDRESS, register)
	if errWrite != nil {
		err = fmt.Errorf("i2cRead error: %s", errWrite.Error())
	}
	return
}

func (mpu *MPU9250) i2cRead2(register byte) (value int16, err error) {

	v, errWrite := mpu.i2cbus.ReadWordFromReg(MPU_ADDRESS, register)
	if errWrite != nil {
		err = fmt.Errorf("MPU9250 Error reading %x: %s\n", register, err.Error())
	} else {
		value = int16(v)
	}
	return
}

func (mpu *MPU9250) memWrite(addr uint16, data *[]byte) error {
	var err error
	var tmp = make([]byte, 2)

	tmp[0] = byte(addr >> 8)
	tmp[1] = byte(addr & 0xFF)

	// Check memory bank boundaries
	if tmp[1]+byte(len(*data)) > MPU_BANK_SIZE {
		return errors.New("Bad address: writing outside of memory bank boundaries")
	}

	err = mpu.i2cbus.WriteToReg(MPU_ADDRESS, MPUREG_BANK_SEL, tmp)
	if err != nil {
		return fmt.Errorf("MPU9250 Error selecting memory bank: %s\n", err.Error())
	}

	err = mpu.i2cbus.WriteToReg(MPU_ADDRESS, MPUREG_MEM_R_W, *data)
	if err != nil {
		return fmt.Errorf("MPU9250 Error writing to the memory bank: %s\n", err.Error())
	}

	return nil
}
