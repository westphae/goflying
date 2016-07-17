package mpu9250

// Approach adapted from the InvenSense DMP 6.1 drivers
// Also referenced https://github.com/brianc118/MPU9250/blob/master/MPU9250.cpp

import (
	"fmt"
	"github.com/kidoman/embd"
	_ "github.com/kidoman/embd/host/all"
	_ "github.com/kidoman/embd/host/rpi"

	"errors"
	"log"
	"math"
	"time"
)

const (
	// Calibration variance tolerances
	MAXGYROVAR  = 10.0
	MAXACCELVAR = 10.0
	BUFSIZE     = 250
)

type MPUData struct {
	G1, G2, G3        float64
	A1, A2, A3        float64
	M1, M2, M3        float64
	GAError, MagError error
	N, NM             int
	T                 time.Time
	DT                time.Duration
}

type MPU9250 struct {
	i2cbus                embd.I2CBus
	scaleGyro, scaleAccel float64 // Max sensor reading for value 2**15-1
	sampleRate            int
	enableMag             bool
	a01, a02, a03         float64   // Accelerometer bias
	g01, g02, g03         float64   // Gyro bias
	mcal1, mcal2, mcal3   int16   // Magnetometer calibration values, uT
	C                     chan *MPUData
	CAvg                  chan *MPUData
	CBuf                  chan *MPUData
	cClose                chan bool
}

func NewMPU9250(sensitivityGyro, sensitivityAccel, sampleRate int, enableMag bool, applyHWOffsets bool) (*MPU9250, error) {
	var mpu = new(MPU9250)

	mpu.sampleRate = sampleRate
	mpu.enableMag = enableMag

	mpu.i2cbus = embd.NewI2CBus(1)

	// Initialization of MPU
	// Reset Device
	if err := mpu.i2cWrite(MPUREG_PWR_MGMT_1, BIT_H_RESET); err != nil {
		return nil, errors.New("Error resetting MPU9250")
	}
	time.Sleep(100 * time.Millisecond) // As in inv_mpu
	// Wake device
	if err := mpu.i2cWrite(MPUREG_PWR_MGMT_1, 0x00); err != nil {
		return nil, errors.New("Error waking MPU9250")
	}
	// Don't let FIFO overwrite DMP data
	if err := mpu.i2cWrite(MPUREG_ACCEL_CONFIG_2, BIT_FIFO_SIZE_1024|0x8); err != nil {
		return nil, errors.New("Error setting up MPU9250")
	}

	/*
		// Invalidate some registers
		// This is done in DMP C drivers, not sure it's needed here
		// Matches gyro_cfg >> 3 & 0x03
		unsigned char gyro_fsr;
		// Matches accel_cfg >> 3 & 0x03
		unsigned char accel_fsr;
		// Enabled sensors. Uses same masks as fifo_en, NOT pwr_mgmt_2.
		unsigned char sensors;
		// Matches config register.
		unsigned char lpf;
		unsigned char clk_src;
		// Sample rate, NOT rate divider.
		unsigned short sample_rate;
		// Matches fifo_en register.
		unsigned char fifo_enable;
		// Matches int enable register.
		unsigned char int_enable;
		// 1 if devices on auxiliary I2C bus appear on the primary.
		unsigned char bypass_mode;
		// 1 if half-sensitivity.
		// NOTE: This doesn't belong here, but everything else in hw_s is const,
		// and this allows us to save some precious RAM.
		 //
		unsigned char accel_half;
		// 1 if device in low-power accel-only mode.
		unsigned char lp_accel_mode;
		// 1 if interrupts are only triggered on motion events.
		unsigned char int_motion_only;
		struct motion_int_cache_s cache;
		// 1 for active low interrupts.
		unsigned char active_low_int;
		// 1 for latched interrupts.
		unsigned char latched_int;
		// 1 if DMP is enabled.
		unsigned char dmp_on;
		// Ensures that DMP will only be loaded once.
		unsigned char dmp_loaded;
		// Sampling rate used when DMP is enabled.
		unsigned short dmp_sample_rate;
		// Compass sample rate.
		unsigned short compass_sample_rate;
		unsigned char compass_addr;
		short mag_sens_adj[3];
	*/

	// Set Gyro and Accel sensitivities
	if err := mpu.SetGyroSensitivity(sensitivityGyro); err != nil {
		log.Println(err)
	}
	if err := mpu.SetAccelSensitivity(sensitivityAccel); err != nil {
		log.Println(err)
	}

	sampRate := byte(1000/mpu.sampleRate - 1)
	// Set LPF to half of sample rate
	if err := mpu.SetLPF(sampRate >> 1); err != nil {
		return nil, err
	}
	// Set sample rate to chosen
	if err := mpu.SetSampleRate(sampRate); err != nil {
		return nil, err
	}
	// Turn off FIFO buffer
	if err := mpu.i2cWrite(MPUREG_INT_ENABLE, 0x00); err != nil {
		return nil, errors.New("Error setting up MPU9250")
	}
	// Turn off FIFO buffer
	//mpu.i2cWrite(MPUREG_FIFO_EN, 0x00)

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
	}

	// Set clock source to PLL
	if err := mpu.i2cWrite(MPUREG_PWR_MGMT_1, INV_CLK_PLL); err != nil {
		return nil, errors.New("Error setting up MPU9250")
	}
	// Turn off all sensors -- Not sure if necessary, but it's in the InvenSense DMP driver
	if err := mpu.i2cWrite(MPUREG_PWR_MGMT_2, 0x63); err != nil {
		return nil, errors.New("Error setting up MPU9250")
	}
	time.Sleep(5 * time.Millisecond)
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

	// Usually we don't want the automatic gyro bias compensation - it pollutes the gyro in a non-inertial frame
	if err := mpu.EnableGyroBiasCal(false); err != nil {
		return nil, err
	}

	go mpu.readGyroAccelRaw()
	//go mpu.readMagRaw()

	time.Sleep(100 * time.Millisecond) // Make sure it's ready
	return mpu, nil
}

// readGyroAccelRaw reads the gyro and accel sensors and totals the values and number of samples
// When Read is called, we will return the averages
func (m *MPU9250) readGyroAccelRaw() {
	var (
		g1, g2, g3, a1, a2, a3, m1, m2, m3                   int16
		avg1, avg2, avg3, ava1, ava2, ava3                   float64
		avm1, avm2, avm3                                     int32
		gaError, magError                                    error
		n, nm                                                float64
		t0, t                                                time.Time
		regmap                                               map[*int16]byte
		//magSampleRate                                        int
	)

	regmap = map[*int16]byte{
		&g1: MPUREG_GYRO_XOUT_H,  &g2: MPUREG_GYRO_YOUT_H,  &g3: MPUREG_GYRO_ZOUT_H,
		&a1: MPUREG_ACCEL_XOUT_H, &a2: MPUREG_ACCEL_YOUT_H, &a3: MPUREG_ACCEL_ZOUT_H,
	}
	//if m.sampleRate > 100 {
	//	magSampleRate = 100
	//} else {
	//	magSampleRate = m.sampleRate
	//}

	m.C = make(chan *MPUData)
	defer close(m.C)
	m.CAvg = make(chan *MPUData)
	defer close(m.CAvg)
	m.CBuf = make(chan *MPUData, BUFSIZE)
	defer close(m.CBuf)
	m.cClose = make(chan bool)
	defer close(m.cClose)

	clock := time.NewTicker(time.Duration(int(1000.0/float32(m.sampleRate)+0.5)) * time.Millisecond)
	defer clock.Stop()

	//clockMag := time.NewTicker(time.Duration(int(1000.0/float32(magSampleRate)+0.5)) * time.Millisecond)
	t0 = time.Now()

	makeMPUData := func() *MPUData {
		d := MPUData{
			G1: (float64(g1) - m.g01) * m.scaleGyro,
			G2: (float64(g2) - m.g02) * m.scaleGyro,
			G3: (float64(g3) - m.g03) * m.scaleGyro,
			A1: (float64(a1) - m.a01) * m.scaleAccel,
			A2: (float64(a2) - m.a01) * m.scaleAccel,
			A3: (float64(a3) - m.a01) * m.scaleAccel,
			M1: float64(int32(m1) * int32(m.mcal1) >> 8),
			M2: float64(int32(m2) * int32(m.mcal2) >> 8),
			M3: float64(int32(m3) * int32(m.mcal3) >> 8),
			GAError: gaError, MagError: magError,
			N: 1, NM: 1, T: t, DT: time.Duration(0),
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
		d := MPUData{}
		if n > 0.5 {
			d.G1 = (avg1/n-m.g01) * m.scaleGyro
			d.G2 = (avg2/n-m.g02) * m.scaleGyro
			d.G3 = (avg3/n-m.g03) * m.scaleGyro
			d.A1 = (ava1/n-m.a01) * m.scaleAccel
			d.A2 = (ava2/n-m.a02) * m.scaleAccel
			d.A3 = (ava3/n-m.a03) * m.scaleAccel
			d.N = int(n+0.5)
			d.T = t
			d.DT = t.Sub(t0)
		} else {
			d.GAError = errors.New("MPU9250 Error: No new accel/gyro values")
		}
		if nm > 0 {
			d.M1 = float64(int32(m1) * int32(m.mcal1) >> 8)
			d.M2 = float64(int32(m2) * int32(m.mcal2) >> 8)
			d.M3 = float64(int32(m3) * int32(m.mcal3) >> 8)
			d.NM = int(nm+0.5); d.T = t; d.DT = t.Sub(t0)
		} else {
			d.MagError = errors.New("MPU9250 Error: No new magnetometer values")
		}
		return &d
	}

	for {
		select {
		case t = <-clock.C: // Read gyro & accel data:
			for p, reg := range regmap {
				*p, gaError = m.i2cRead2(reg)
				if gaError != nil {
					log.Println("MPU9250 Warning: error reading gyro/accel")
				}
			}
			// Update accumulated values and increment count of gyro/accel readings
			avg1 += float64(g1); avg2 += float64(g2); avg3 += float64(g3)
			ava1 += float64(a1); ava2 += float64(a2); ava3 += float64(a3)
			avm1 += int32(m1); avm2 += int32(m2); avm3 += int32(m3)
			n++
		case m.C<- makeMPUData(): // Send the latest values
		case m.CBuf<- makeMPUData():
		case m.CAvg<- makeAvgMPUData(): // Send the averages
			avg1, avg2, avg3 = 0, 0, 0
			ava1, ava2, ava3 = 0, 0, 0
			avm1, avm2, avm3 = 0, 0, 0
			n = 0
			t0 = t
		case <-m.cClose: // Stop the goroutine, ease up on the CPU
			break
		}
	}
}

// readMagRaw reads all sensors and totals the values and number of samples
// When Read is called, we will return the averages
func (m *MPU9250) readMagRaw() {
	var (
		m1, m2, m3, m4 int16
		err            error
		regmap         = map[*int16]byte{
			&m1: MPUREG_EXT_SENS_DATA_00,
			&m2: MPUREG_EXT_SENS_DATA_02,
			&m3: MPUREG_EXT_SENS_DATA_04,
			&m4: MPUREG_EXT_SENS_DATA_06,
		}
	)

	clock := time.NewTicker(time.Duration(int(1000.0/float32(m.sampleRate)+0.5)) * time.Millisecond)
	defer clock.Stop()

	for m.enableMag {
		<-clock.C
		// Read magnetometer data:
		if err := m.i2cWrite(MPUREG_I2C_SLV0_ADDR, AK8963_I2C_ADDR|READ_FLAG); err != nil {
			log.Printf("MPU9250 Error: couldn't set AK8963 address for reading: %s", err.Error())
		}
		//I2C slave 0 register address from where to begin data transfer
		if err := m.i2cWrite(MPUREG_I2C_SLV0_REG, AK8963_HXL); err != nil {
			log.Printf("MPU9250 Error: couldn't set AK8963 read register: %s", err.Error())
		}
		//Tell AK8963 that we will read 7 bytes
		if err := m.i2cWrite(MPUREG_I2C_SLV0_CTRL, 0x87); err != nil {
			log.Printf("MPU9250 Error: couldn't communicate with AK8963: %s", err.Error())
		}

		// Read the actual data
		for p, reg := range regmap {
			*p, err = m.i2cRead2(reg)
			if err != nil {
				log.Println("MPU9250 Warning: error reading magnetometer")
			}
		}

		// Test validity of magnetometer data
		if (byte(m1&0xFF)&AKM_DATA_READY) == 0x00 && (byte(m1&0xFF)&AKM_DATA_OVERRUN) != 0x00 {
			log.Println("MPU9250 Mag data not ready or overflow")
			log.Printf("MPU9250 m1 LSB: %X\n", byte(m1&0xFF))
			return // Don't update the accumulated values
		}

		if (byte((m4>>8)&0xFF) & AKM_OVERFLOW) != 0x00 {
			log.Println("MPU9250 Mag data overflow")
			log.Printf("MPU9250 m4 MSB: %X\n", byte((m1>>8)&0xFF))
			return // Don't update the accumulated values
		}

		// Update values and increment count of magnetometer readings
		//m.m1 += int16(m1) * m.mcal1 >> 8
		//m.m2 += int16(m2) * m.mcal2 >> 8
		//m.m3 += int16(m3) * m.mcal3 >> 8
		//m.nm++
	}
}

func (m *MPU9250) CloseMPU() {
	// Nothing to do bitwise for the 9250?
	m.cClose<- true
}

// CalibrateGyro does a live calibration of the gyro, sampling over (dur int) seconds.
// It is only intended to be run intelligently so that it isn't called when the sensor is in a non-inertial state.
func (m *MPU9250) Calibrate(dur int) error {
	var (
		n             float64 = float64(dur * m.sampleRate)
		g11, g12, g13 int32 // Accumulators for calculating mean drifts
		g21, g22, g23 int64 // Accumulators for calculating stdev drifts
		a11, a12, a13 int32 // Accumulators for calculating mean drifts
		a21, a22, a23 int64 // Accumulators for calculating stdev drifts
	)

	clock := time.NewTicker(time.Duration(int(1000.0/float32(m.sampleRate)+0.5)) * time.Millisecond)
	defer clock.Stop()

	for i := float64(0); i < n; i++ {
		<-clock.C

		g1, err := m.i2cRead2(MPUREG_GYRO_XOUT_H)
		if err != nil {
			return errors.New("CalibrationGyro: sensor error during calibration")
		}
		g2, err := m.i2cRead2(MPUREG_GYRO_YOUT_H)
		if err != nil {
			return errors.New("CalibrationGyro: sensor error during calibration")
		}
		g3, err := m.i2cRead2(MPUREG_GYRO_ZOUT_H)
		if err != nil {
			return errors.New("CalibrationGyro: sensor error during calibration")
		}
		g11 += int32(g1)
		g12 += int32(g2)
		g13 += int32(g3)
		g21 += int64(g1) * int64(g1)
		g22 += int64(g2) * int64(g2)
		g23 += int64(g3) * int64(g3)

		a1, err := m.i2cRead2(MPUREG_ACCEL_XOUT_H)
		if err != nil {
			return errors.New("CalibrationAccel: sensor error during calibration")
		}
		a2, err := m.i2cRead2(MPUREG_ACCEL_YOUT_H)
		if err != nil {
			return errors.New("CalibrationAccel: sensor error during calibration")
		}
		a3, err := m.i2cRead2(MPUREG_ACCEL_ZOUT_H)
		if err != nil {
			return errors.New("CalibrationAccel: sensor error during calibration")
		}
		a3 -= int16(1.0 / m.scaleAccel)
		a11 += int32(a1)
		a12 += int32(a2)
		a13 += int32(a3)
		a21 += int64(a1) * int64(a1)
		a22 += int64(a2) * int64(a2)
		a23 += int64(a3) * int64(a3)
	}

	// Too much variance in the gyro readings means it was moving too much for a good calibration
	log.Printf("MPU9250 gyro calibration variance: %f %f %f\n", (float64(g21-int64(g11)*int64(g11)/int64(n)) * m.scaleGyro * m.scaleGyro / n),
		(float64(g22-int64(g12)*int64(g12)/int64(n)) * m.scaleGyro * m.scaleGyro / n),
		(float64(g23-int64(g13)*int64(g13)/int64(n)) * m.scaleGyro * m.scaleGyro / n))
	log.Printf("MPU9250 accel calibration variance: %f %f %f\n", (float64(a21-int64(a11)*int64(a11)/int64(n)) * m.scaleAccel * m.scaleAccel / n),
		(float64(a22-int64(a12)*int64(a12)/int64(n)) * m.scaleAccel * m.scaleAccel / n),
		(float64(a23-int64(a13)*int64(a13)/int64(n)) * m.scaleAccel * m.scaleAccel / n))
	if (float64(g21-int64(g11)*int64(g11)/int64(n))*m.scaleGyro*m.scaleGyro > n*MAXGYROVAR) ||
		(float64(g22-int64(g12)*int64(g12)/int64(n))*m.scaleGyro*m.scaleGyro > n*MAXGYROVAR) ||
		(float64(g23-int64(g13)*int64(g13)/int64(n))*m.scaleGyro*m.scaleGyro > n*MAXGYROVAR) ||
		(float64(a21-int64(a11)*int64(a11)/int64(n))*m.scaleAccel*m.scaleAccel > n*MAXACCELVAR) ||
		(float64(a22-int64(a12)*int64(a12)/int64(n))*m.scaleAccel*m.scaleAccel > n*MAXACCELVAR) ||
		(float64(a23-int64(a13)*int64(a13)/int64(n))*m.scaleAccel*m.scaleAccel > n*MAXACCELVAR) {
		return errors.New("MPU9250 CalibrationAll: sensor was not inertial during calibration")
	}

	m.g01 = float64(g11) / n
	m.g02 = float64(g12) / n
	m.g03 = float64(g13) / n
	m.a01 = float64(a11) / n
	m.a02 = float64(a12) / n
	m.a03 = float64(a13) / n

	log.Printf("MPU9250 Gyro calibration: %d, %d, %d\n", m.g01, m.g02, m.g03)
	log.Printf("MPU9250 Accel calibration: %d, %d, %d\n", m.a01, m.a02, m.a03)
	return nil
}

func (mpu *MPU9250) SetSampleRate(rate byte) (err error) {
	errWrite := mpu.i2cWrite(MPUREG_SMPLRT_DIV, byte(rate)) // Set sample rate to chosen
	if errWrite != nil {
		err = fmt.Errorf("MPU9250 Error: Couldn't set sample rate: %s", errWrite.Error())
	}
	return
}

func (mpu *MPU9250) SetLPF(rate byte) (err error) {
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
		err = fmt.Errorf("MPU9250 Error: couldn't set LPF: %s", errWrite.Error())
	}
	return
}

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

func (mpu *MPU9250) SampleRate() int {
	return mpu.sampleRate
}

func (mpu *MPU9250) MagEnabled() bool {
	return mpu.enableMag
}

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
		mpu.a01 = float64(a0x >> 1)
		mpu.a02 = float64(a0y >> 1)
		mpu.a03 = float64(a0z >> 1)
	case 8:
		mpu.a01 = float64(a0x)
		mpu.a02 = float64(a0y)
		mpu.a03 = float64(a0z)
	case 4:
		mpu.a01 = float64(a0x << 1)
		mpu.a02 = float64(a0y << 1)
		mpu.a03 = float64(a0z << 1)
	case 2:
		mpu.a01 = float64(a0x << 2)
		mpu.a02 = float64(a0y << 2)
		mpu.a03 = float64(a0z << 2)
	default:
		return fmt.Errorf("MPU9250 Error: %d is not a valid acceleration sensitivity", sensitivityAccel)
	}

	log.Printf("MPU9250 Accel bias read: %d %d %d\n", mpu.a01, mpu.a02, mpu.a03)
	return nil
}

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
		mpu.g01 = float64(g0x >> 1)
		mpu.g02 = float64(g0y >> 1)
		mpu.g03 = float64(g0z >> 1)
	case 1000:
		mpu.g01 = float64(g0x)
		mpu.g02 = float64(g0y)
		mpu.g03 = float64(g0z)
	case 500:
		mpu.g01 = float64(g0x << 1)
		mpu.g02 = float64(g0y << 1)
		mpu.g03 = float64(g0z << 1)
	case 250:
		mpu.g01 = float64(g0x << 2)
		mpu.g02 = float64(g0y << 2)
		mpu.g03 = float64(g0z << 2)
	default:
		return fmt.Errorf("MPU9250 Error: %d is not a valid gyro sensitivity", sensitivityGyro)
	}

	log.Printf("MPU9250 Gyro  bias read: %d %d %d\n", mpu.g01, mpu.g02, mpu.g03)
	return nil
}

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

	mpu.mcal1 = int16(mcal1) + 128
	mpu.mcal2 = int16(mcal2) + 128
	mpu.mcal3 = int16(mcal3) + 128

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

	log.Printf("MPU9250 Mag bias: %d %d %d\n", mpu.mcal1, mpu.mcal2, mpu.mcal3)
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
