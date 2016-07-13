package mpu9250

// Bit banging borrowed courtesy of
// https://github.com/brianc118/MPU9250/blob/master/MPU9250.cpp

import (
	"github.com/kidoman/embd"
	_ "github.com/kidoman/embd/host/all"
	_ "github.com/kidoman/embd/host/rpi"

	"fmt"
	"math"
	"time"
	"errors"
	"sync"
)

const (
	MPU_ADDRESS               = 0x68
	MPUREG_XG_OFFS_TC         = 0x00
	MPUREG_YG_OFFS_TC         = 0x01
	MPUREG_ZG_OFFS_TC         = 0x02
	MPUREG_X_FINE_GAIN        = 0x03
	MPUREG_Y_FINE_GAIN        = 0x04
	MPUREG_Z_FINE_GAIN        = 0x05
	MPUREG_XA_OFFS_H          = 0x06
	MPUREG_XA_OFFS_L          = 0x07
	MPUREG_YA_OFFS_H          = 0x08
	MPUREG_YA_OFFS_L          = 0x09
	MPUREG_ZA_OFFS_H          = 0x0A
	MPUREG_ZA_OFFS_L          = 0x0B
	MPUREG_PRODUCT_ID         = 0x0C
	MPUREG_SELF_TEST_X        = 0x0D
	MPUREG_SELF_TEST_Y        = 0x0E
	MPUREG_SELF_TEST_Z        = 0x0F
	MPUREG_SELF_TEST_A        = 0x10
	MPUREG_XG_OFFS_USRH       = 0x13
	MPUREG_XG_OFFS_USRL       = 0x14
	MPUREG_YG_OFFS_USRH       = 0x15
	MPUREG_YG_OFFS_USRL       = 0x16
	MPUREG_ZG_OFFS_USRH       = 0x17
	MPUREG_ZG_OFFS_USRL       = 0x18
	MPUREG_SMPLRT_DIV         = 0x19
	MPUREG_CONFIG             = 0x1A
	MPUREG_GYRO_CONFIG        = 0x1B
	MPUREG_ACCEL_CONFIG       = 0x1C
	MPUREG_ACCEL_CONFIG_2     = 0x1D
	MPUREG_LP_ACCEL_ODR       = 0x1E
	MPUREG_MOT_THR            = 0x1F
	MPUREG_FIFO_EN            = 0x23
	MPUREG_I2C_MST_CTRL       = 0x24
	MPUREG_I2C_SLV0_ADDR      = 0x25
	MPUREG_I2C_SLV0_REG       = 0x26
	MPUREG_I2C_SLV0_CTRL      = 0x27
	MPUREG_I2C_SLV1_ADDR      = 0x28
	MPUREG_I2C_SLV1_REG       = 0x29
	MPUREG_I2C_SLV1_CTRL      = 0x2A
	MPUREG_I2C_SLV2_ADDR      = 0x2B
	MPUREG_I2C_SLV2_REG       = 0x2C
	MPUREG_I2C_SLV2_CTRL      = 0x2D
	MPUREG_I2C_SLV3_ADDR      = 0x2E
	MPUREG_I2C_SLV3_REG       = 0x2F
	MPUREG_I2C_SLV3_CTRL      = 0x30
	MPUREG_I2C_SLV4_ADDR      = 0x31
	MPUREG_I2C_SLV4_REG       = 0x32
	MPUREG_I2C_SLV4_DO        = 0x33
	MPUREG_I2C_SLV4_CTRL      = 0x34
	MPUREG_I2C_SLV4_DI        = 0x35
	MPUREG_I2C_MST_STATUS     = 0x36
	MPUREG_INT_PIN_CFG        = 0x37
	MPUREG_INT_ENABLE         = 0x38
	MPUREG_ACCEL_XOUT_H       = 0x3B
	MPUREG_ACCEL_XOUT_L       = 0x3C
	MPUREG_ACCEL_YOUT_H       = 0x3D
	MPUREG_ACCEL_YOUT_L       = 0x3E
	MPUREG_ACCEL_ZOUT_H       = 0x3F
	MPUREG_ACCEL_ZOUT_L       = 0x40
	MPUREG_TEMP_OUT_H         = 0x41
	MPUREG_TEMP_OUT_L         = 0x42
	MPUREG_GYRO_XOUT_H        = 0x43
	MPUREG_GYRO_XOUT_L        = 0x44
	MPUREG_GYRO_YOUT_H        = 0x45
	MPUREG_GYRO_YOUT_L        = 0x46
	MPUREG_GYRO_ZOUT_H        = 0x47
	MPUREG_GYRO_ZOUT_L        = 0x48
	MPUREG_EXT_SENS_DATA_00   = 0x49
	MPUREG_EXT_SENS_DATA_01   = 0x4A
	MPUREG_EXT_SENS_DATA_02   = 0x4B
	MPUREG_EXT_SENS_DATA_03   = 0x4C
	MPUREG_EXT_SENS_DATA_04   = 0x4D
	MPUREG_EXT_SENS_DATA_05   = 0x4E
	MPUREG_EXT_SENS_DATA_06   = 0x4F
	MPUREG_EXT_SENS_DATA_07   = 0x50
	MPUREG_EXT_SENS_DATA_08   = 0x51
	MPUREG_EXT_SENS_DATA_09   = 0x52
	MPUREG_EXT_SENS_DATA_10   = 0x53
	MPUREG_EXT_SENS_DATA_11   = 0x54
	MPUREG_EXT_SENS_DATA_12   = 0x55
	MPUREG_EXT_SENS_DATA_13   = 0x56
	MPUREG_EXT_SENS_DATA_14   = 0x57
	MPUREG_EXT_SENS_DATA_15   = 0x58
	MPUREG_EXT_SENS_DATA_16   = 0x59
	MPUREG_EXT_SENS_DATA_17   = 0x5A
	MPUREG_EXT_SENS_DATA_18   = 0x5B
	MPUREG_EXT_SENS_DATA_19   = 0x5C
	MPUREG_EXT_SENS_DATA_20   = 0x5D
	MPUREG_EXT_SENS_DATA_21   = 0x5E
	MPUREG_EXT_SENS_DATA_22   = 0x5F
	MPUREG_EXT_SENS_DATA_23   = 0x60
	MPUREG_I2C_SLV0_DO        = 0x63
	MPUREG_I2C_SLV1_DO        = 0x64
	MPUREG_I2C_SLV2_DO        = 0x65
	MPUREG_I2C_SLV3_DO        = 0x66
	MPUREG_I2C_MST_DELAY_CTRL = 0x67
	MPUREG_SIGNAL_PATH_RESET  = 0x68
	MPUREG_MOT_DETECT_CTRL    = 0x69
	MPUREG_USER_CTRL          = 0x6A
	MPUREG_PWR_MGMT_1         = 0x6B
	MPUREG_PWR_MGMT_2         = 0x6C
	MPUREG_BANK_SEL           = 0x6D
	MPUREG_MEM_START_ADDR     = 0x6E
	MPUREG_MEM_R_W            = 0x6F
	MPUREG_DMP_CFG_1          = 0x70
	MPUREG_DMP_CFG_2          = 0x71
	MPUREG_FIFO_COUNTH        = 0x72
	MPUREG_FIFO_COUNTL        = 0x73
	MPUREG_FIFO_R_W           = 0x74
	MPUREG_WHOAMI             = 0x75
	MPUREG_XA_OFFSET_H        = 0x77
	MPUREG_XA_OFFSET_L        = 0x78
	MPUREG_YA_OFFSET_H        = 0x7A
	MPUREG_YA_OFFSET_L        = 0x7B
	MPUREG_ZA_OFFSET_H        = 0x7D
	MPUREG_ZA_OFFSET_L        = 0x7E
	/* ---- AK8963 Reg In MPU9250 ----------------------------------------------- */
	AK8963_I2C_ADDR  = 0x0C //0x18
	AK8963_Device_ID = 0x48
	AK8963_MAX_SAMPLE_RATE = 0x64 // 100 Hz
	// Read-only Reg
	AK8963_WIA  = 0x00
	AK8963_INFO = 0x01
	AK8963_ST1  = 0x02
	AK8963_HXL  = 0x03
	AK8963_HXH  = 0x04
	AK8963_HYL  = 0x05
	AK8963_HYH  = 0x06
	AK8963_HZL  = 0x07
	AK8963_HZH  = 0x08
	AK8963_ST2  = 0x09
	// Write/Read Reg
	AK8963_CNTL1  = 0x0A
	AK8963_CNTL2  = 0x0B
	AK8963_ASTC   = 0x0C
	AK8963_TS1    = 0x0D
	AK8963_TS2    = 0x0E
	AK8963_I2CDIS = 0x0F
	// Read-only Reg ( ROM )
	AK8963_ASAX = 0x10
	AK8963_ASAY = 0x11
	AK8963_ASAZ = 0x12
	// Configuration bits mpu9250
	BIT_SLEEP                  = 0x40
	BIT_H_RESET                = 0x80
	BITS_CLKSEL                = 0x07
	MPU_CLK_SEL_PLLGYROX       = 0x01
	MPU_CLK_SEL_PLLGYROZ       = 0x03
	MPU_EXT_SYNC_GYROX         = 0x02
	BITS_FS_250DPS             = 0x00
	BITS_FS_500DPS             = 0x08
	BITS_FS_1000DPS            = 0x10
	BITS_FS_2000DPS            = 0x18
	BITS_FS_2G                 = 0x00
	BITS_FS_4G                 = 0x08
	BITS_FS_8G                 = 0x10
	BITS_FS_16G                = 0x18
	BITS_FS_MASK               = 0x18
	BITS_DLPF_CFG_256HZ_NOLPF2 = 0x00
	BITS_DLPF_CFG_188HZ        = 0x01
	BITS_DLPF_CFG_98HZ         = 0x02
	BITS_DLPF_CFG_42HZ         = 0x03
	BITS_DLPF_CFG_20HZ         = 0x04
	BITS_DLPF_CFG_10HZ         = 0x05
	BITS_DLPF_CFG_5HZ          = 0x06
	BITS_DLPF_CFG_2100HZ_NOLPF = 0x07
	BITS_DLPF_CFG_MASK         = 0x07
	BIT_INT_ANYRD_2CLEAR       = 0x10
	BIT_RAW_RDY_EN             = 0x01
	BIT_I2C_IF_DIS             = 0x10

	// Misc
	READ_FLAG = 0x80
	MPU_BANK_SIZE = 0xFF
	CFG_MOTION_BIAS = 0x4B8 // Enable/disable gyro bias compensation
	BIT_FIFO_SIZE_1024 = 0x40 // FIFO buffer size
	BIT_AUX_IF_EN uint8 = 0x20
	BIT_BYPASS_EN = 0x02
	AKM_POWER_DOWN = 0x00
	BIT_I2C_READ = 0x80
	BIT_SLAVE_EN = 0x80
	AKM_SINGLE_MEASUREMENT = 0x01
	INV_CLK_PLL = 0x01
	AK89xx_FSR = 4915
	AKM_DATA_READY = 0x01
	AKM_DATA_OVERRUN = 0x02
	AKM_OVERFLOW = 0x80


	/* = ---- Sensitivity --------------------------------------------------------- */

	MPU9250M_4800uT                       = 0.6            // 0.6 uT/LSB
	MPU9250T_85degC                       = 0.002995177763 // 0.002995177763 degC/LSB
	Magnetometer_Sensitivity_Scale_Factor = 0.15
)

type MPU9250 struct {
	i2cbus                embd.I2CBus
	scaleGyro, scaleAccel float64 // Max sensor reading for value 2**15-1
	sampleRate 	      int
	n, nm                 float64 // Number of samples taken since last read
	g1, g2, g3            int32 // Gyro accumulated values, rad/s
	a1, a2, a3            int32 // Accel accumulated values, G
	m1, m2, m3            int32 // Magnetometer accumulated values, uT
	a01, a02, a03	      int16 // Accelerometer bias
	g01, g02, g03	      int16 // Gyro bias
	mcal1, mcal2, mcal3   int32 // Magnetometer calibration values, uT
	m 			sync.Mutex
}

func NewMPU9250(sensitivityGyro, sensitivityAccel, sampleRate int, applyHWOffsets bool) (*MPU9250, error) {
	var mpu = new(MPU9250)
	var sensGyro, sensAccel byte
	mpu.sampleRate = sampleRate

	switch {
	case sensitivityGyro>1000:
		sensGyro = BITS_FS_2000DPS
		mpu.scaleGyro = 2000.0 / float64(math.MaxInt16)
	case sensitivityGyro>500:
		sensGyro = BITS_FS_1000DPS
		mpu.scaleGyro = 1000.0 / float64(math.MaxInt16)
	case sensitivityGyro>250:
		sensGyro = BITS_FS_500DPS
		mpu.scaleGyro = 500.0 / float64(math.MaxInt16)
	default:
		sensGyro = BITS_FS_250DPS
		mpu.scaleGyro = 250.0 / float64(math.MaxInt16)
	}

	switch {
	case sensitivityAccel>8:
		sensAccel = BITS_FS_16G
		mpu.scaleAccel = 16.0 / float64(math.MaxInt16)
	case sensitivityAccel>4:
		sensAccel = BITS_FS_8G
		mpu.scaleAccel = 8.0 / float64(math.MaxInt16)
	case sensitivityAccel>2:
		sensAccel = BITS_FS_4G
		mpu.scaleAccel = 4.0 / float64(math.MaxInt16)
	default:
		sensAccel = BITS_FS_2G
		mpu.scaleAccel = 2.0 / float64(math.MaxInt16)
	}

	mpu.i2cbus = embd.NewI2CBus(1)

	// Initialization of MPU
	// Reset Device
	if err := mpu.i2cWrite(MPUREG_PWR_MGMT_1, BIT_H_RESET); err != nil {
		return nil, errors.New("Error resetting MPU9250")
	}
	time.Sleep(100*time.Millisecond)		// As in inv_mpu
	// Wake device
	if err := mpu.i2cWrite(MPUREG_PWR_MGMT_1, 0x00); err != nil {
		return nil, errors.New("Error waking MPU9250")
	}
	// Don't let FIFO overwrite DMP data
	if err := mpu.i2cWrite(MPUREG_ACCEL_CONFIG_2, BIT_FIFO_SIZE_1024 | 0x8); err != nil {
		return nil, errors.New("Error setting up MPU9250")
	}

	// Invalidate some registers
	/*
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
	if err := mpu.i2cWrite(MPUREG_GYRO_CONFIG, sensGyro); err != nil {
		return nil, errors.New("Error setting MPU9250 gyro sensitivity")
	}
	if err := mpu.i2cWrite(MPUREG_ACCEL_CONFIG, sensAccel); err != nil {
		return nil, errors.New("Error setting MPU9250 accel sensitivity")
	}
	sampRate := byte(1000/mpu.sampleRate-1)
	// Set LPF to half of sample rate
	mpu.SetLPF(sampRate >> 1)
	// Set sample rate to chosen
	mpu.SetSampleRate(sampRate)
	// Turn off FIFO buffer
	if err := mpu.i2cWrite(MPUREG_INT_ENABLE, 0x00); err != nil {
		return nil, errors.New("Error setting up MPU9250")
	}
	// Turn off FIFO buffer
	//mpu.i2cWrite(MPUREG_FIFO_EN, 0x00)

	// Set up compass
	if err := mpu.ReadMagCalibration(); err != nil {
		return nil, errors.New("Error reading calibration from magnetometer")
	}

	// Set up AK8963 master mode, master clock and ES bit
	if err := mpu.i2cWrite(MPUREG_I2C_MST_CTRL, 0x40); err != nil {
		return nil, errors.New("Error setting up AK8963")
	}
	// Slave 0 reads from AK8963
	if err := mpu.i2cWrite(MPUREG_I2C_SLV0_ADDR, BIT_I2C_READ | AK8963_I2C_ADDR); err != nil {
		return nil, errors.New("Error setting up AK8963")
	}
	// Compass reads start at this register
	if err := mpu.i2cWrite(MPUREG_I2C_SLV0_REG, AK8963_ST1); err != nil {
		return nil, errors.New("Error setting up AK8963")
	}
	// Enable 8-byte reads on slave 0
	if err := mpu.i2cWrite(MPUREG_I2C_SLV0_CTRL, BIT_SLAVE_EN | 8); err != nil {
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
	if err := mpu.i2cWrite(MPUREG_I2C_SLV1_CTRL, BIT_SLAVE_EN | 1); err != nil {
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

	// Set clock source to PLL
	if err := mpu.i2cWrite(MPUREG_PWR_MGMT_1, INV_CLK_PLL); err != nil {
		return nil, errors.New("Error setting up MPU9250")
	}
	// Turn off all sensors -- Not sure if necessary
	if err := mpu.i2cWrite(MPUREG_PWR_MGMT_2, 0x63); err != nil {
		return nil, errors.New("Error setting up MPU9250")
	}
	time.Sleep(5 * time.Millisecond)
	// Turn on all gyro, all accel
	if err := mpu.i2cWrite(MPUREG_PWR_MGMT_2, 0x00); err != nil {
		return nil, errors.New("Error setting up MPU9250")
	}

	if applyHWOffsets {
		if err := mpu.ReadAccelBias(sensAccel); err != nil {
			return nil, err
		}
		if err := mpu.ReadGyroBias(sensGyro); err != nil {
			return nil, err
		}
	}

	// Usually we don't want the automatic gyro bias compensation - it pollutes the gyro in a non-inertial frame
	if err := mpu.EnableGyroBiasCal(false); err != nil {
		return nil, err
	}

	go mpu.readMPURaw()

	return mpu, nil
}

// readMPURaw reads all sensors and totals the values and number of samples
// When Read is called, we will return the averages
func (m *MPU9250) readMPURaw() {
	var g1, g2, g3, a1, a2, a3, m1, m2, m3, m4 int16
	var err error

	clock := time.NewTicker(time.Duration(int(1000.0/float32(m.sampleRate)+0.5)) * time.Millisecond)

	for {
		<-clock.C
		m.m.Lock()
		// Read gyro data:
		g1, err = m.i2cRead2(MPUREG_GYRO_XOUT_H)
		if err != nil {
			fmt.Println("Warning: error reading gyro")
			goto readMagData
		}
		g2, err = m.i2cRead2(MPUREG_GYRO_YOUT_H)
		if err != nil {
			fmt.Println("Warning: error reading gyro")
			goto readMagData
		}
		g3, err = m.i2cRead2(MPUREG_GYRO_ZOUT_H)
		if err != nil {
			fmt.Println("Warning: error reading gyro")
			goto readMagData
		}

		// Read accelerometer data:
		a1, err = m.i2cRead2(MPUREG_ACCEL_XOUT_H)
		if err != nil {
			fmt.Println("Warning: error reading accelerometer")
			goto readMagData
		}
		a2, err = m.i2cRead2(MPUREG_ACCEL_YOUT_H)
		if err != nil {
			fmt.Println("Warning: error reading accelerometer")
			goto readMagData
		}
		a3, err = m.i2cRead2(MPUREG_ACCEL_ZOUT_H)
		if err != nil {
			fmt.Println("Warning: error reading accelerometer")
			goto readMagData
		}

		// Update values and increment count of gyro/accel readings
		m.g1 += int32(g1 - m.g01)
		m.g2 += int32(g2 - m.g02)
		m.g3 += int32(g3 - m.g03)
		m.a1 += int32(a1 - m.a01)
		m.a2 += int32(a2 - m.a02)
		m.a3 += int32(a3 - m.a03)
		m.n += 1.0

		readMagData:
		// Read magnetometer data:
		m.i2cWrite(MPUREG_I2C_SLV0_ADDR, AK8963_I2C_ADDR | READ_FLAG)
		m.i2cWrite(MPUREG_I2C_SLV0_REG, AK8963_HXL) //I2C slave 0 register address from where to begin data transfer
		m.i2cWrite(MPUREG_I2C_SLV0_CTRL, 0x87)      //Read 7 bytes from the magnetometer

		m1, err = m.i2cRead2(MPUREG_EXT_SENS_DATA_00)
		if err != nil {
			fmt.Println("Warning: error reading magnetometer")
			return	// Don't update the accumulated values
		}
		m2, err = m.i2cRead2(MPUREG_EXT_SENS_DATA_02)
		if err != nil {
			fmt.Println("Warning: error reading magnetometer")
			return	// Don't update the accumulated values
		}
		m3, err = m.i2cRead2(MPUREG_EXT_SENS_DATA_04)
		if err != nil {
			fmt.Println("Warning: error reading magnetometer")
			return	// Don't update the accumulated values
		}
		m4, err = m.i2cRead2(MPUREG_EXT_SENS_DATA_06)
		if err != nil {
			fmt.Println("Warning: error reading magnetometer")
			return	// Don't update the accumulated values
		}

		if (byte(m1 & 0xFF) & AKM_DATA_READY) == 0x00 && (byte(m1 & 0xFF) & AKM_DATA_OVERRUN) != 0x00 {
			fmt.Println("Mag data not ready or overflow")
			fmt.Printf("m1 LSB: %X\n", byte(m1 & 0xFF))
			return	// Don't update the accumulated values
		}

		if (byte((m4 >> 8) & 0xFF) & AKM_OVERFLOW) != 0x00 {
			fmt.Println("Mag data overflow")
			fmt.Printf("m4 MSB: %X\n", byte((m1 >> 8) & 0xFF))
			return	// Don't update the accumulated values
		}

		m.m1 += (int32(m1) * m.mcal1 >> 8)
		m.m2 += (int32(m2) * m.mcal2 >> 8)
		m.m3 += (int32(m3) * m.mcal3 >> 8)

		m.nm += 1.0
		m.m.Unlock()
	}
}

func (m *MPU9250) Read() (int64, float64, float64, float64, float64, float64, float64, float64, float64, float64, error, error) {
	m.m.Lock()
	var g1, g2, g3, a1, a2, a3, m1, m2, m3 float64
	var gaError, magError error

	if m.n > 0 {
		g1, g2, g3 = float64(m.g1) / m.n * m.scaleGyro, float64(m.g2) / m.n * m.scaleGyro, float64(m.g3) / m.n * m.scaleGyro
		a1, a2, a3 = float64(m.a1) / m.n * m.scaleAccel, float64(m.a2) / m.n * m.scaleAccel, float64(m.a3) / m.n * m.scaleAccel
		gaError = nil
	} else {
		gaError = errors.New("MPU9250 Read: error reading gyro/accel")
	}
	if m.nm > 0 {
		m1, m2, m3 = float64(m.m1) / m.nm, float64(m.m2) / m.nm, float64(m.m3) / m.nm
		magError = nil
	} else {
		magError = errors.New("MPU9250 Read: error reading magnetometer")
	}
	t := time.Now().UnixNano()

	m.g1, m.g2, m.g3 = 0, 0, 0
	m.a1, m.a2, m.a3 = 0, 0, 0
	m.m1, m.m2, m.m3 = 0, 0, 0
	m.n, m.nm = 0, 0
	m.m.Unlock()

	return t, g1, g2, g3, a1, a2, a3, m1, m2, m3, gaError, magError
}

func (m *MPU9250) CloseMPU() {
	return // Nothing to do for the 9250?
}

func (mpu *MPU9250) SetSampleRate(rate byte) {
	mpu.i2cWrite(MPUREG_SMPLRT_DIV, byte(rate)) // Set sample rate to chosen
}

func (mpu*MPU9250) SetLPF(rate byte) {
	var r byte
	switch {
	case rate >= 188:
		r = BITS_DLPF_CFG_188HZ
	case rate >= 98:
		r = BITS_DLPF_CFG_98HZ
	case rate >= 42:
		r = BITS_DLPF_CFG_42HZ;
	case rate >= 20:
		r = BITS_DLPF_CFG_20HZ
	case rate >= 10:
		r = BITS_DLPF_CFG_10HZ
	default:
		r = BITS_DLPF_CFG_5HZ
	}

	mpu.i2cWrite(MPUREG_CONFIG, r)
}

func (mpu *MPU9250) EnableGyroBiasCal(enable bool) (error) {
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

func (mpu *MPU9250) ReadAccelBias(sensAccel byte) error {
	a0x, err := mpu.i2cRead2(MPUREG_XA_OFFSET_H)
	if err != nil {
		return errors.New("ReadAccelBias error reading chip")
	}
	a0y, err := mpu.i2cRead2(MPUREG_YA_OFFSET_H)
	if err != nil {
		return errors.New("ReadAccelBias error reading chip")
	}
	a0z, err := mpu.i2cRead2(MPUREG_ZA_OFFSET_H)
	if err != nil {
		return errors.New("ReadAccelBias error reading chip")
	}

	switch sensAccel {
	case BITS_FS_16G:
		fmt.Println("16 G")
		mpu.a01 = a0x >> 1
		mpu.a02 = a0y >> 1
		mpu.a03 = a0z >> 1
	case BITS_FS_4G:
		fmt.Println("4 G")
		mpu.a01 = a0x << 1
		mpu.a02 = a0y << 1
		mpu.a03 = a0z << 1
	case BITS_FS_2G:
		fmt.Println("2 G")
		mpu.a01 = a0x << 2
		mpu.a02 = a0y << 2
		mpu.a03 = a0z << 2
	default:
		mpu.a01 = a0x
		mpu.a02 = a0y
		mpu.a03 = a0z
	}

	fmt.Printf("Accel bias read: %d %d %d\n", mpu.a01, mpu.a02, mpu.a03)
	return nil
}

func (mpu *MPU9250) ReadGyroBias(sensGyro byte) error {
	g0x, err := mpu.i2cRead2(MPUREG_XG_OFFS_USRH)
	if err != nil {
		return errors.New("ReadGyroBias error reading chip")
	}
	g0y, err := mpu.i2cRead2(MPUREG_YG_OFFS_USRH)
	if err != nil {
		return errors.New("ReadGyroBias error reading chip")
	}
	g0z, err := mpu.i2cRead2(MPUREG_ZG_OFFS_USRH)
	if err != nil {
		return errors.New("ReadGyroBias error reading chip")
	}

	switch sensGyro {
	case BITS_FS_2000DPS:
		fmt.Println("2000 DPS")
		mpu.g01 = g0x >> 1
		mpu.g02 = g0y >> 1
		mpu.g03 = g0z >> 1
	case BITS_FS_500DPS:
		fmt.Println("500 DPS")
		mpu.g01 = g0x << 1
		mpu.g02 = g0y << 1
		mpu.g03 = g0z << 1
	case BITS_FS_250DPS:
		fmt.Println("250 DPS")
		mpu.g01 = g0x << 2
		mpu.g02 = g0y << 2
		mpu.g03 = g0z << 2
	default:
		mpu.g01 = g0x
		mpu.g02 = g0y
		mpu.g03 = g0z
	}

	fmt.Printf("Gyro  bias read: %d %d %d\n", mpu.g01, mpu.g02, mpu.g03)
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
	if err = mpu.i2cWrite(MPUREG_I2C_SLV0_CTRL, AK8963_CNTL1)  ; err != nil {
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

	mpu.mcal1 = int32(mcal1 + 128)
	mpu.mcal2 = int32(mcal2 + 128)
	mpu.mcal3 = int32(mcal3 + 128)

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
	if err = mpu.i2cWrite(MPUREG_USER_CTRL, tmp | BIT_AUX_IF_EN); err != nil {
		return errors.New("ReadMagCalibration error reading chip")
	}
	time.Sleep(3 * time.Millisecond)
	if err = mpu.i2cWrite(MPUREG_INT_PIN_CFG, 0x00); err != nil {
		return errors.New("ReadMagCalibration error reading chip")
	}

	fmt.Printf("Mag   bias: %d %d %d\n", mpu.mcal1, mpu.mcal2, mpu.mcal3)
	return nil
}

func (mpu *MPU9250) i2cWrite(register, value byte) error {

	if err := mpu.i2cbus.WriteByteToReg(MPU_ADDRESS, register, value); err != nil {
		fmt.Printf("Error writing %x to %x: %s\n", value, register, err.Error())
		return errors.New("i2cWrite error")
	}

	time.Sleep(time.Millisecond)
	return nil
}

func (mpu *MPU9250) i2cRead(register byte) (uint8, error) {

	value, err := mpu.i2cbus.ReadByteFromReg(MPU_ADDRESS, register)
	if err != nil {
		return 0, errors.New("i2cRead error")
	}
	return value, nil
}

func (mpu *MPU9250) i2cRead2(register byte) (int16, error) {

	value, err := mpu.i2cbus.ReadWordFromReg(MPU_ADDRESS, register)
	if err != nil {
		fmt.Printf("Error reading %x: %s\n", register, err.Error())
		return 0, errors.New("i2cRead2 error")
	}
	return int16(value), nil
}

func (mpu *MPU9250) memWrite(addr uint16, data *[]byte) (error) {
	var err error
	var tmp = make([]byte, 2)

	tmp[0] = byte(addr >> 8)
	tmp[1] = byte(addr & 0xFF)

	// Check memory bank boundaries
	if tmp[1] + byte(len(*data)) > MPU_BANK_SIZE {
		return errors.New("Bad address: writing outside of memory bank boundaries")
	}

	err = mpu.i2cbus.WriteToReg(MPU_ADDRESS, MPUREG_BANK_SEL, tmp)
	if err != nil {
		fmt.Printf("Error selecting memory bank: %s\n", err.Error())
		return err
	}

	err = mpu.i2cbus.WriteToReg(MPU_ADDRESS, MPUREG_MEM_R_W, *data)
	if err != nil {
		fmt.Printf("Error writing to the memory bank: %s\n", err.Error())
		return err
	}

	return nil
}
