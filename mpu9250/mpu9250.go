package mpu9250

// Approach adapted from the InvenSense DMP 6.1 drivers
// Also referenced https://github.com/brianc118/MPU9250/blob/master/MPU9250.cpp

import (
	"encoding/json"
	"fmt"
	"github.com/kidoman/embd"
	_ "github.com/kidoman/embd/host/all"
	_ "github.com/kidoman/embd/host/rpi"

	"errors"
	"log"
	"math"
	"time"
	"os"
)

const (
	// Calibration variance tolerances
	//TODO westphae: would be nice to have some mathematical reasoning for this
	MAXGYROVAR  = 0.1
	MAXACCELVAR = 0.1
	BUFSIZE     = 250
	SCALEMAG    = 9830.0/65536
	CALDATALOCATION = "/etc/mpu9250cal.json"
)

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
	A01, A02, A03    float64 // Accelerometer bias
	G01, G02, G03    float64 // Gyro bias
	M01, M02, M03    float64 // Magnetometer bias
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
	fd, err := os.OpenFile(CALDATALOCATION, os.O_CREATE|os.O_WRONLY|os.O_TRUNC, os.FileMode(0644))
	if err != nil {
		log.Printf("MPU9250: Error saving calibration data to %s: %s", CALDATALOCATION, err.Error())
		return
	}
	defer fd.Close()
	calData, err := json.Marshal(d)
	if err != nil {
		log.Printf("MPU9250: Error marshaling calibration data: %s", err)
		return
	}
	fd.Write(calData)
	log.Println("MPU9250: Wrote calibration data:")
	log.Println(string(calData))
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
	fd, rerr := os.Open(CALDATALOCATION)
	if rerr != nil {
		err = fmt.Errorf(errstr, CALDATALOCATION, rerr.Error())
		return
	}
	defer fd.Close()
	buf := make([]byte, 1024)
	count, rerr := fd.Read(buf)
	if rerr != nil {
		err = fmt.Errorf(errstr, CALDATALOCATION, rerr.Error())
		return
	}
	rerr = json.Unmarshal(buf[0:count], d)
	if rerr != nil {
		err = fmt.Errorf(errstr, CALDATALOCATION, rerr.Error())
		return
	}
	log.Println("MPU9250: read calibration data:")
	log.Println(*d)
	return
}

type MPU9250 struct {
	i2cbus                embd.I2CBus
	scaleGyro, scaleAccel float64 // Max sensor reading for value 2**15-1
	sampleRate            int
	enableMag             bool
	                      mpuCalData
	mcal1, mcal2, mcal3   float64   // Hardware magnetometer calibration values, uT
	C                     <-chan *MPUData
	CAvg                  <-chan *MPUData
	CBuf                  <-chan *MPUData
	CCal                  chan<- int
	CCalResult            <-chan error
	cClose                chan bool
}

func NewMPU9250(sensitivityGyro, sensitivityAccel, sampleRate int, enableMag bool, applyHWOffsets bool) (*MPU9250, error) {
	var mpu = new(MPU9250)
	if err := mpu.mpuCalData.load(); err != nil {
		log.Println(err)
		mpu.mpuCalData.reset()
	}

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

	go mpu.readSensors()
	time.Sleep(50 * time.Millisecond) // Make sure it's ready

	return mpu, nil
}

// readGyroAccelRaw reads the gyro and accel sensors and totals the values and number of samples
// When Read is called, we will return the averages
func (m *MPU9250) readSensors() {
	var (
		g1, g2, g3, a1, a2, a3, m1, m2, m3, m4, tmp          int16 // Current values
		avg1, avg2, avg3, ava1, ava2, ava3, avtmp            float64 // Accumulators for averages
		avm1, avm2, avm3                                     int32
		g11, g12, g13, a11, a12, a13                         float64 // Accumulators for calculating mean drifts
		g21, g22, g23, a21, a22, a23                         float64 // Accumulators for calculating stdev drifts
		n, nm, nc, i                                         float64
		gaError, magError                                    error
		t0, t, t0m, tm                                       time.Time
		magSampleRate                                        int
	)

	acRegMap := map[*int16]byte{
		&g1: MPUREG_GYRO_XOUT_H, &g2: MPUREG_GYRO_YOUT_H, &g3: MPUREG_GYRO_ZOUT_H,
		&a1: MPUREG_ACCEL_XOUT_H, &a2: MPUREG_ACCEL_YOUT_H, &a3: MPUREG_ACCEL_ZOUT_H,
		&tmp: MPUREG_TEMP_OUT_H,
	}
	magRegMap := map[*int16]byte{
		&m1: MPUREG_EXT_SENS_DATA_00, &m2: MPUREG_EXT_SENS_DATA_02, &m3: MPUREG_EXT_SENS_DATA_04, &m4: MPUREG_EXT_SENS_DATA_06,
	}

	if m.sampleRate > 100 {
		magSampleRate = 100
	} else {
		magSampleRate = m.sampleRate
	}

	cC := make(chan *MPUData)
	defer close(cC)
	m.C = cC
	cAvg := make(chan *MPUData)
	defer close(cAvg)
	m.CAvg = cAvg
	cBuf := make(chan *MPUData, BUFSIZE)
	defer close(cBuf)
	m.CBuf = cBuf
	cCal := make(chan int)
	defer close(cCal)
	m.CCal = cCal
	cCalResult := make(chan error)
	defer close(cCalResult)
	m.CCalResult = cCalResult
	m.cClose = make(chan bool)
	defer close(m.cClose)

	clock := time.NewTicker(time.Duration(int(1000.0/float32(m.sampleRate)+0.5)) * time.Millisecond)
	//TODO westphae: use the clock to record actual time instead of a timer
	defer clock.Stop()

	clockMag := time.NewTicker(time.Duration(int(1000.0/float32(magSampleRate)+0.5)) * time.Millisecond)
	t0 = time.Now()
	t0m = time.Now()

	makeMPUData := func() *MPUData {
		mm1 := float64(m1) * m.mcal1 - m.M01
		mm2 := float64(m2) * m.mcal2 - m.M02
		mm3 := float64(m3) * m.mcal3 - m.M03
		d := MPUData{
			G1: (float64(g1) - m.G01) * m.scaleGyro,
			G2: (float64(g2) - m.G02) * m.scaleGyro,
			G3: (float64(g3) - m.G03) * m.scaleGyro,
			A1: (float64(a1) - m.A01) * m.scaleAccel,
			A2: (float64(a2) - m.A02) * m.scaleAccel,
			A3: (float64(a3) - m.A03) * m.scaleAccel,
			M1: m.Ms11*mm1 + m.Ms12*mm2 + m.Ms13*mm3,
			M2: m.Ms21*mm1 + m.Ms22*mm2 + m.Ms23*mm3,
			M3: m.Ms31*mm1 + m.Ms32*mm2 + m.Ms33*mm3,
			Temp: (float64(tmp)/340+36.53),
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
		mm1 := float64(avm1) * m.mcal1 / nm - m.M01
		mm2 := float64(avm2) * m.mcal2 / nm - m.M02
		mm3 := float64(avm3) * m.mcal3 / nm - m.M03
		d := MPUData{}
		if n > 0.5 {
			d.G1 = (avg1/n-m.G01) * m.scaleGyro
			d.G2 = (avg2/n-m.G02) * m.scaleGyro
			d.G3 = (avg3/n-m.G03) * m.scaleGyro
			d.A1 = (ava1/n-m.A01) * m.scaleAccel
			d.A2 = (ava2/n-m.A02) * m.scaleAccel
			d.A3 = (ava3/n-m.A03) * m.scaleAccel
			d.Temp = (float64(avtmp)/n)/340+36.53
			d.N = int(n+0.5); d.T = t; d.DT = t.Sub(t0)
		} else {
			d.GAError = errors.New("MPU9250 Error: No new accel/gyro values")
		}
		if nm > 0 {
			d.M1 = m.Ms11 *mm1 + m.Ms12 *mm2 + m.Ms13 *mm3
			d.M2 = m.Ms21 *mm1 + m.Ms22 *mm2 + m.Ms23 *mm3
			d.M3 = m.Ms31 *mm1 + m.Ms32 *mm2 + m.Ms33 *mm3
			d.NM = int(nm+0.5); d.TM = tm; d.DTM = t.Sub(t0m)
		} else {
			d.MagError = errors.New("MPU9250 Error: No new magnetometer values")
		}
		return &d
	}

	for {
		select {
		case t = <-clock.C: // Read accel/gyro data:
			for p, reg := range acRegMap {
				*p, gaError = m.i2cRead2(reg)
				if gaError != nil {
					log.Println("MPU9250 Warning: error reading gyro/accel")
				}
			}
			// Update accumulated values and increment count of gyro/accel readings
			avg1 += float64(g1); avg2 += float64(g2); avg3 += float64(g3)
			ava1 += float64(a1); ava2 += float64(a2); ava3 += float64(a3)
			avtmp += float64(tmp)
			avm1 += int32(m1); avm2 += int32(m2); avm3 += int32(m3)
			n++
			if i < nc-0.5 { // Then we're doing a calibration
				g11 += float64(g1)
				g12 += float64(g2)
				g13 += float64(g3)
				g21 += float64(g1) * float64(g1)
				g22 += float64(g2) * float64(g2)
				g23 += float64(g3) * float64(g3)
				//TODO westphae: don't calibrate accel!
				a11 += float64(a1)
				a12 += float64(a2)
				a13 += float64(a3) + 1/m.scaleAccel
				a21 += float64(a1) * float64(a1)
				a22 += float64(a2) * float64(a2)
				a23 += (float64(a3) + 1/m.scaleAccel) * (float64(a3) - 1/m.scaleAccel)
				i++
			} else if nc > 0.5 { // Then we're finished with a calibration
				vg1 := (g21-g11*g11/nc) * m.scaleGyro * m.scaleGyro / nc
				vg2 := (g22-g12*g12/nc) * m.scaleGyro * m.scaleGyro / nc
				vg3 := (g23-g13*g13/nc) * m.scaleGyro * m.scaleGyro / nc
				va1 := (a21-a11*a11/nc) * m.scaleAccel * m.scaleAccel / nc
				va2 := (a22-a12*a12/nc) * m.scaleAccel * m.scaleAccel / nc
				va3 := (a23-a13*a13/nc) * m.scaleAccel * m.scaleAccel / nc
				log.Printf("MPU9250 Calibration: %.0f values collected\n", i)
				log.Printf("MPU9250 Calibration: gyro variance:  %f %f %f\n", vg1, vg2, vg3)
				log.Printf("MPU9250 Calibration: accel variance: %f %f %f\n", va1, va2, va3)
				// Could check that it's not nearly level here too
				if      a11/nc*m.scaleAccel > 0.5 || a11/nc*m.scaleAccel < -0.5 ||
					a12/nc*m.scaleAccel > 0.5 || a12/nc*m.scaleAccel < -0.5 ||
					a13/nc*m.scaleAccel > 0.5 || a13/nc*m.scaleAccel < -0.5 {
					cCalResult<- fmt.Errorf("MPU9250 Calibration Error: accel is maxing out: %6f %6f %6f",
						a11/nc*m.scaleAccel, a12/nc*m.scaleAccel, a13/nc*m.scaleAccel)
				} else if vg1 > MAXGYROVAR || vg2 > MAXGYROVAR || vg3 > MAXGYROVAR ||
					va1 > MAXACCELVAR || va2 > MAXACCELVAR || va3 > MAXACCELVAR {
					cCalResult<- errors.New("MPU9250 Calibration Error: sensor was not inertial during calibration")
				} else {
					m.G01 = g11 / nc
					m.G02 = g12 / nc
					m.G03 = g13 / nc
					m.A01 = a11 / nc
					m.A02 = a12 / nc
					m.A03 = a13 / nc

					log.Printf("MPU9250 Gyro Calibration: %6f, %6f, %6f\n", m.G01 *m.scaleGyro, m.G02 *m.scaleGyro, m.G03 *m.scaleGyro)
					log.Printf("MPU9250 Accel Calibration: %6f, %6f, %6f\n", m.A01 *m.scaleAccel, m.A02 *m.scaleAccel, m.A03 *m.scaleAccel)
					cCalResult <- nil
				}
				g11, g12, g13, g21, g22, g23 = 0, 0, 0, 0, 0, 0
				a11, a12, a13, a21, a22, a23 = 0, 0, 0, 0, 0, 0
				nc = 0
			}
		case tm = <-clockMag.C: // Read magnetometer data:
			if m.enableMag {
				// Set AK8963 to slave0 for reading
				if err := m.i2cWrite(MPUREG_I2C_SLV0_ADDR, AK8963_I2C_ADDR | READ_FLAG); err != nil {
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
				for p, reg := range magRegMap {
					*p, magError = m.i2cRead2(reg)
					if magError != nil {
						log.Println("MPU9250 Warning: error reading magnetometer")
					}
				}

				// Test validity of magnetometer data
				if (byte(m1 & 0xFF) & AKM_DATA_READY) == 0x00 && (byte(m1 & 0xFF) & AKM_DATA_OVERRUN) != 0x00 {
					log.Println("MPU9250 Mag data not ready or overflow")
					log.Printf("MPU9250 m1 LSB: %X\n", byte(m1 & 0xFF))
					continue // Don't update the accumulated values
				}

				if (byte((m4 >> 8) & 0xFF) & AKM_OVERFLOW) != 0x00 {
					log.Println("MPU9250 Mag data overflow")
					log.Printf("MPU9250 m4 MSB: %X\n", byte((m1 >> 8) & 0xFF))
					continue // Don't update the accumulated values
				}

				// Update values and increment count of magnetometer readings
				avm1 += int32(m1)
				avm2 += int32(m2)
				avm3 += int32(m3)
				nm++
			}
		case cC<- makeMPUData(): // Send the latest values
		case cBuf<- makeMPUData():
		case cAvg<- makeAvgMPUData(): // Send the averages
			avg1, avg2, avg3 = 0, 0, 0
			ava1, ava2, ava3 = 0, 0, 0
			avm1, avm2, avm3 = 0, 0, 0
			avtmp = 0
			n, nm = 0, 0
			t0, t0m = t, tm
		case dur := <-cCal:
			nc = float64(dur * m.sampleRate) // nc>0 triggers sampling for a calibration
			i = 0
		case <-m.cClose: // Stop the goroutine, ease up on the CPU
			break
		}
	}
}

func (m *MPU9250) CloseMPU() {
	// Nothing to do bitwise for the 9250?
	m.cClose<- true
}

func (mpu *MPU9250) SetSampleRate(rate byte) (err error) {
	errWrite := mpu.i2cWrite(MPUREG_SMPLRT_DIV, byte(rate)) // Set sample rate to chosen
	if errWrite != nil {
		err = fmt.Errorf("MPU9250 Error: Couldn't set sample rate: %s", errWrite.Error())
	}
	return
}

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

	log.Printf("MPU9250 Accel bias read: %d %d %d\n", mpu.A01, mpu.A02, mpu.A03)
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

	log.Printf("MPU9250 Gyro  bias read: %d %d %d\n", mpu.G01, mpu.G02, mpu.G03)
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

	log.Printf("Raw mag calibrations: %d %d %d\n", mcal1, mcal2, mcal3)
	mpu.mcal1 = float64(int16(mcal1) + 128) / 256 * SCALEMAG
	mpu.mcal2 = float64(int16(mcal2) + 128) / 256 * SCALEMAG
	mpu.mcal3 = float64(int16(mcal3) + 128) / 256 * SCALEMAG

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

	log.Printf("MPU9250 Mag bias: %f %f %f\n", mpu.mcal1, mpu.mcal2, mpu.mcal3)
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

// TODO westphae: write something to perform a guided magnetometer calibration
func CalbrateMagnetometer() {

}