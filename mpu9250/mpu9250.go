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

	READ_FLAG = 0x80

	/* = ---- Sensitivity --------------------------------------------------------- */

	MPU9250M_4800uT                       = 0.6            // 0.6 uT/LSB
	MPU9250T_85degC                       = 0.002995177763 // 0.002995177763 degC/LSB
	Magnetometer_Sensitivity_Scale_Factor = 0.15
)

type MPU9250 struct {
	i2cbus                embd.I2CBus
	freq                  float32 // Update frequency in Hz
	scaleGyro, scaleAccel float32 // Max sensor reading for value 2**15-1
	zeroGyro,  zeroAccel  float32 // Max sensor reading for value 2**15-1
	n                     float32 // Number of samples taken since last read
	g1, g2, g3            float32 // Gyro accumulated values, rad/s
	a1, a2, a3            float32 // Accel accumulated values, G
	m1, m2, m3            float32 // Magnetometer accumulated values, uT
	mcal1, mcal2, mcal3   float32 // Magnetometer calibration values, uT
}

func NewMPU9250(freq float32, sensitivityGyro, sensitivityAccel int) *MPU9250 {
	var mpu = new(MPU9250)
	var sensGyro, sensAccel byte
	mpu.freq = freq

	switch sensitivityGyro {
	case 2000:
		sensGyro = BITS_FS_2000DPS
		mpu.zeroGyro = float32(1000)
		mpu.scaleGyro = float32(2000) / float32(math.MaxInt16)
	case 1000:
		sensGyro = BITS_FS_1000DPS
		mpu.zeroGyro = float32(500)
		mpu.scaleGyro = float32(1000) / float32(math.MaxInt16)
	case 500:
		sensGyro = BITS_FS_500DPS
		mpu.zeroGyro = float32(250)
		mpu.scaleGyro = float32(500) / float32(math.MaxInt16)
	case 250:
	default:
		sensGyro = BITS_FS_250DPS
		mpu.zeroGyro = float32(125)
		mpu.scaleGyro = float32(250) / float32(math.MaxInt16)
	}

	switch sensitivityAccel {
	case 16:
		sensAccel = BITS_FS_16G
		mpu.zeroAccel = float32(8)
		mpu.scaleAccel = float32(16) / float32(math.MaxInt16)
	case 8:
		sensAccel = BITS_FS_8G
		mpu.zeroAccel = float32(4)
		mpu.scaleAccel = float32(8) / float32(math.MaxInt16)
	case 4:
		sensAccel = BITS_FS_4G
		mpu.zeroAccel = float32(2)
		mpu.scaleAccel = float32(4) / float32(math.MaxInt16)
	case 2:
	default:
		sensAccel = BITS_FS_2G
		mpu.zeroAccel = float32(1)
		mpu.scaleAccel = float32(2) / float32(math.MaxInt16)
	}

	mpu.i2cbus = embd.NewI2CBus(1)

	// Initialization of MPU
	mpu.i2cWrite(BIT_H_RESET, MPUREG_PWR_MGMT_1) // Reset Device
	mpu.i2cWrite(0x01, MPUREG_PWR_MGMT_1)        // Clock Source
	mpu.i2cWrite(0x00, MPUREG_PWR_MGMT_2)        // Enable Acc & Gyro
	//i2cWrite(my_low_pass_filter, MPUREG_CONFIG)         	// Use DLPF set Gyroscope bandwidth 184Hz, temperature bandwidth 188Hz
	mpu.i2cWrite(sensGyro, MPUREG_GYRO_CONFIG)              // +-250dps
	mpu.i2cWrite(sensAccel, MPUREG_ACCEL_CONFIG)            // +-8G
	mpu.i2cWrite(BITS_DLPF_CFG_98HZ, MPUREG_ACCEL_CONFIG_2) // Set Acc Data Rates, Enable Acc LPF , Bandwidth 184Hz
	mpu.i2cWrite(0x30, MPUREG_INT_PIN_CFG)                  //
	//mpu.i2cWrite(0x40, MPUREG_I2C_MST_CTRL)   		// I2C Speed 348 kHz
	//mpu.i2cWrite(0x20, MPUREG_USER_CTRL)      		// Enable AUX
	mpu.i2cWrite(0x20, MPUREG_USER_CTRL)    // I2C Master mode
	mpu.i2cWrite(0x0D, MPUREG_I2C_MST_CTRL) //  I2C configuration multi-master  IIC 400KHz

	mpu.i2cWrite(AK8963_I2C_ADDR, MPUREG_I2C_SLV0_ADDR) //Set the I2C slave addres of AK8963 and set for write.
	//mpu.i2cWrite(0x09, MPUREG_I2C_SLV4_CTRL)
	//mpu.i2cWrite(0x81, MPUREG_I2C_MST_DELAY_CTRL)		//Enable I2C delay

	mpu.i2cWrite(AK8963_CNTL2, MPUREG_I2C_SLV0_REG) //I2C slave 0 register address from where to begin data transfer
	mpu.i2cWrite(0x01, MPUREG_I2C_SLV0_DO)          // Reset AK8963
	mpu.i2cWrite(0x81, MPUREG_I2C_SLV0_CTRL)        //Enable I2C and set 1 byte

	mpu.i2cWrite(AK8963_CNTL1, MPUREG_I2C_SLV0_REG) //I2C slave 0 register address from where to begin data transfer
	mpu.i2cWrite(0x16, MPUREG_I2C_SLV0_DO)          // Register value to 100Hz continuous measurement in 16bit
	mpu.i2cWrite(0x81, MPUREG_I2C_SLV0_CTRL)        //Enable I2C and set 1 byte

	mpu.readMagCal()
	go mpu.readMPURaw()

	return mpu
}

// readMPURaw reads all sensors and totals the values and number of samples
// When Read is called, we will return the averages
func (m *MPU9250) readMPURaw() {
	clock := time.NewTicker(time.Duration(int(1000/m.freq+0.5)) * time.Millisecond)

	for {
		<-clock.C
		// Read gyro data:
		m.g1 += float32(m.i2cRead(MPUREG_GYRO_XOUT_H))
		m.g2 += float32(m.i2cRead(MPUREG_GYRO_YOUT_H))
		m.g3 += float32(m.i2cRead(MPUREG_GYRO_ZOUT_H))

		// Read accelerometer data:
		m.a1 += float32(m.i2cRead(MPUREG_ACCEL_XOUT_H))
		m.a2 += float32(m.i2cRead(MPUREG_ACCEL_YOUT_H))
		m.a3 += float32(m.i2cRead(MPUREG_ACCEL_ZOUT_H))

		// Read magnetometer data:
		m.i2cWrite(AK8963_I2C_ADDR|READ_FLAG, MPUREG_I2C_SLV0_ADDR)
		m.i2cWrite(AK8963_HXL, MPUREG_I2C_SLV0_REG) //I2C slave 0 register address from where to begin data transfer
		m.i2cWrite(0x87, MPUREG_I2C_SLV0_CTRL)      //Read 7 bytes from the magnetometer

		m.m1 += float32(m.i2cRead(MPUREG_EXT_SENS_DATA_00))
		m.m2 += float32(m.i2cRead(MPUREG_EXT_SENS_DATA_02))
		m.m3 += float32(m.i2cRead(MPUREG_EXT_SENS_DATA_04))
		_ = m.i2cRead(MPUREG_EXT_SENS_DATA_06)

		m.n += 1.0
	}
}

func (m *MPU9250) Read() (int64, float32, float32, float32, float32, float32, float32, float32, float32, float32) {
	fmt.Printf("%.0f values read\n", m.n)
	g1 := m.g1/m.n*m.scaleGyro-m.zeroGyro
	g2 := m.g2/m.n*m.scaleGyro-m.zeroGyro
	g3 := m.g3/m.n*m.scaleGyro-m.zeroGyro
	a1 := m.a1/m.n*m.scaleAccel-m.zeroAccel
	a2 := m.a2/m.n*m.scaleAccel-m.zeroAccel
	a3 := m.a3/m.n*m.scaleAccel-m.zeroAccel
	m1, m2, m3 := m.m1*m.mcal1/m.n, m.m2*m.mcal2/m.n, m.m3*m.mcal3/m.n
	t := time.Now().UnixNano()
	m.g1, m.g2, m.g3, m.a1, m.a2, m.a3, m.m1, m.m2, m.m3, m.n = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
	return t, g1, g2, g3, a1, a2, a3, m1, m2, m3
}

func (m *MPU9250) readMagCal() {
	m.i2cWrite(AK8963_I2C_ADDR|READ_FLAG, MPUREG_I2C_SLV0_ADDR) //Set the I2C slave addres of AK8963 and set for read.
	m.i2cWrite(AK8963_ASAX, MPUREG_I2C_SLV0_REG)                //I2C slave 0 register address from where to begin data transfer
	m.i2cWrite(0x83, MPUREG_I2C_SLV0_CTRL)                      //Read 3 bytes from the magnetometer

	m.mcal1 += ((float32(m.i2cRead(MPUREG_EXT_SENS_DATA_00))-128)/256 + 1) * Magnetometer_Sensitivity_Scale_Factor
	m.mcal2 += ((float32(m.i2cRead(MPUREG_EXT_SENS_DATA_02))-128)/256 + 1) * Magnetometer_Sensitivity_Scale_Factor
	m.mcal3 += ((float32(m.i2cRead(MPUREG_EXT_SENS_DATA_04))-128)/256 + 1) * Magnetometer_Sensitivity_Scale_Factor
	fmt.Printf("Mag calibration: %+6.f %+6.f %+6.f\n", m.mcal1, m.mcal2, m.mcal3)
}

func (m *MPU9250) CloseMPU() {
	return // Nothing to do for the 9250?
}
func (m *MPU9250) Freq() float32 {
	return m.freq
}

func (mpu *MPU9250) i2cWrite(value, register byte) {

	if err := mpu.i2cbus.WriteByteToReg(MPU_ADDRESS, register, value); err != nil {
		fmt.Printf("Error writing %x to %x: %s\n", value, register, err.Error())
	}
	time.Sleep(time.Millisecond)
}

func (mpu *MPU9250) i2cRead(register byte) uint16 {

	value, err := mpu.i2cbus.ReadWordFromReg(MPU_ADDRESS, register)
	if err != nil {
		fmt.Printf("Error reading %x: %s\n", register, err.Error())
	}
	return value
}
