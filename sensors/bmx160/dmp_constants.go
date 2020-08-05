package bmx160

const (

/*
 BMM150_W_NORMAL_SELF_TEST_YZ_FAIL    =1
 BMM150_W_NORMAL_SELF_TEST_XZ_FAIL    =2
 BMM150_W_NORMAL_SELF_TEST_Z_FAIL     =3
 BMM150_W_NORMAL_SELF_TEST_XY_FAIL    =4
 BMM150_W_NORMAL_SELF_TEST_Y_FAIL     =5
 BMM150_W_NORMAL_SELF_TEST_X_FAIL     =6
 BMM150_W_NORMAL_SELF_TEST_XYZ_FAIL   =7
 BMM150_W_ADV_SELF_TEST_FAIL          =8
*/

/**\name CHIP ID & SOFT RESET VALUES      */
 BMM150_CHIP_ID                       =0x32
 BMM150_SET_SOFT_RESET                =0x82

/**\name POWER MODE DEFINTIONS      */
 BMM150_NORMAL_MODE                   =0x00
 BMM150_FORCED_MODE                   =0x01
 BMM150_SLEEP_MODE                    =0x03
 BMM150_SUSPEND_MODE                  =0x04

/**\name I2C ADDRESS       */
 BMM150_DEFAULT_I2C_ADDRESS           =0x10
 BMM150_I2C_ADDRESS_CSB_LOW_SDO_HIGH  =0x11
 BMM150_I2C_ADDRESS_CSB_HIGH_SDO_LOW  =0x12
 BMM150_I2C_ADDRESS_CSB_HIGH_SDO_HIGH =0x13

/**\name Power mode settings  */
 BMM150_POWER_CNTRL_DISABLE           =0x00
 BMM150_POWER_CNTRL_ENABLE            =0x01

/**\name Sensor delay time settings  */
 BMM150_SOFT_RESET_DELAY              =1
 BMM150_NORMAL_SELF_TEST_DELAY        =2
 BMM150_START_UP_TIME                 =3
 BMM150_ADV_SELF_TEST_DELAY           =4

/**\name ENABLE/DISABLE DEFINITIONS  */
 BMM150_XY_CHANNEL_ENABLE             =0x00
 BMM150_XY_CHANNEL_DISABLE            =0x03

/**\name Register Address */
 BMM150_CHIP_ID_ADDR                  =0x40
 BMM150_DATA_X_LSB                    =0x42
 BMM150_DATA_READY_STATUS             =0x48
 BMM150_INTERRUPT_STATUS              =0x4A
 BMM150_POWER_CONTROL_ADDR            =0x4B
 BMM150_OP_MODE_ADDR                  =0x4C
 BMM150_INT_CONFIG_ADDR               =0x4D
 BMM150_AXES_ENABLE_ADDR              =0x4E
 BMM150_LOW_THRESHOLD_ADDR            =0x4F
 BMM150_HIGH_THRESHOLD_ADDR           =0x50
 BMM150_REP_XY_ADDR                   =0x51
 BMM150_REP_Z_ADDR                    =0x52

/**\name Macros to select the sensor settings to be set by the user
 * These values are internal for API implementation. Don't relate this to
 * data sheet. */
/* 
BMM150_DATA_RATE_SEL                 =1
 BMM150_CONTROL_MEASURE_SEL           =1 << 1
 BMM150_XY_REP_SEL                    =1 << 2
 BMM150_Z_REP_SEL                     =1 << 3
 BMM150_DRDY_PIN_EN_SEL               =1 << 4
 BMM150_INT_PIN_EN_SEL                =1 << 5
 BMM150_DRDY_POLARITY_SEL             =1 << 6
 BMM150_INT_LATCH_SEL                 =1 << 7
 BMM150_INT_POLARITY_SEL              =1 << 8
 BMM150_DATA_OVERRUN_INT_SEL          =1 << 9
 BMM150_OVERFLOW_INT_SEL              =1 << 10
 BMM150_HIGH_THRESHOLD_INT_SEL        =1 << 11
 BMM150_LOW_THRESHOLD_INT_SEL         =1 << 12
 BMM150_LOW_THRESHOLD_SETTING_SEL     =1 << 13
 BMM150_HIGH_THRESHOLD_SETTING_SEL    =1 << 14
*/
/**\name DATA RATE DEFINITIONS  */
 BMM150_DATA_RATE_10HZ                =0x00
 BMM150_DATA_RATE_02HZ                =0x01
 BMM150_DATA_RATE_06HZ                =0x02
 BMM150_DATA_RATE_08HZ                =0x03
 BMM150_DATA_RATE_15HZ                =0x04
 BMM150_DATA_RATE_20HZ                =0x05
 BMM150_DATA_RATE_25HZ                =0x06
 BMM150_DATA_RATE_30HZ                =0x07

/**\name TRIM REGISTERS      */
/* Trim Extended Registers */
 BMM150_DIG_X1                        =0x5D
 BMM150_DIG_Y1                        =0x5E
 BMM150_DIG_Z4_LSB                    =0x62
 BMM150_DIG_Z4_MSB                    =0x63
 BMM150_DIG_X2                        =0x64
 BMM150_DIG_Y2                        =0x65
 BMM150_DIG_Z2_LSB                    =0x68
 BMM150_DIG_Z2_MSB                    =0x69
 BMM150_DIG_Z1_LSB                    =0x6A
 BMM150_DIG_Z1_MSB                    =0x6B
 BMM150_DIG_XYZ1_LSB                  =0x6C
 BMM150_DIG_XYZ1_MSB                  =0x6D
 BMM150_DIG_Z3_LSB                    =0x6E
 BMM150_DIG_Z3_MSB                    =0x6F
 BMM150_DIG_XY2                       =0x70
 BMM150_DIG_XY1                       =0x71

/**\name Threshold interrupt setting macros for x,y,z axes selection */
 BMM150_THRESHOLD_X                   =0x06
 BMM150_THRESHOLD_Y                   =0x05
 BMM150_THRESHOLD_Z                   =0x03

/**\name User configurable interrupt setting macros */
 BMM150_INT_ENABLE                    =0x01
 BMM150_INT_DISABLE                   =0x00
 BMM150_ACTIVE_HIGH_POLARITY          =0x01
 BMM150_ACTIVE_LOW_POLARITY           =0x00
 BMM150_LATCHED                       =0x01
 BMM150_NON_LATCHED                   =0x00

/**\name Interrupt status */
 BMM150_LOW_THRESHOLD_INT_X           =0x0001
 BMM150_LOW_THRESHOLD_INT_Y           =0x0002
 BMM150_LOW_THRESHOLD_INT_Z           =0x0004
 BMM150_HIGH_THRESHOLD_INT_X          =0x0008
 BMM150_HIGH_THRESHOLD_INT_Y          =0x0010
 BMM150_HIGH_THRESHOLD_INT_Z          =0x0020
 BMM150_DATA_OVERFLOW_INT             =0x0040
 BMM150_DATA_OVERRUN_INT              =0x0080
 BMM150_DATA_READY_INT                =0x0100

/**\name Macros for bit masking */
 BMM150_PWR_CNTRL_MSK                 =0x01

 BMM150_CONTROL_MEASURE_MSK           =0x38
 BMM150_CONTROL_MEASURE_POS           =0x03

 BMM150_POWER_CONTROL_BIT_MSK         =0x01
 BMM150_POWER_CONTROL_BIT_POS         =0x00

 BMM150_OP_MODE_MSK                   =0x06
 BMM150_OP_MODE_POS                   =0x01

 BMM150_ODR_MSK                       =0x38
 BMM150_ODR_POS                       =0x03

 BMM150_DATA_X_MSK                    =0xF8
 BMM150_DATA_X_POS                    =0x03

 BMM150_DATA_Y_MSK                    =0xF8
 BMM150_DATA_Y_POS                    =0x03

 BMM150_DATA_Z_MSK                    =0xFE
 BMM150_DATA_Z_POS                    =0x01

 BMM150_DATA_RHALL_MSK                =0xFC
 BMM150_DATA_RHALL_POS                =0x02

 BMM150_SELF_TEST_MSK                 =0x01

 BMM150_ADV_SELF_TEST_MSK             =0xC0
 BMM150_ADV_SELF_TEST_POS             =0x06

 BMM150_DRDY_EN_MSK                   =0x80
 BMM150_DRDY_EN_POS                   =0x07

 BMM150_INT_PIN_EN_MSK                =0x40
 BMM150_INT_PIN_EN_POS                =0x06

 BMM150_DRDY_POLARITY_MSK             =0x04
 BMM150_DRDY_POLARITY_POS             =0x02

 BMM150_INT_LATCH_MSK                 =0x02
 BMM150_INT_LATCH_POS                 =0x01

 BMM150_INT_POLARITY_MSK              =0x01

 BMM150_DATA_OVERRUN_INT_MSK          =0x80
 BMM150_DATA_OVERRUN_INT_POS          =0x07

 BMM150_OVERFLOW_INT_MSK              =0x40
 BMM150_OVERFLOW_INT_POS              =0x06

 BMM150_HIGH_THRESHOLD_INT_MSK        =0x38
 BMM150_HIGH_THRESHOLD_INT_POS        =0x03

 BMM150_LOW_THRESHOLD_INT_MSK         =0x07

 BMM150_DRDY_STATUS_MSK               =0x01

/**\name OVERFLOW DEFINITIONS  */
 BMM150_XYAXES_FLIP_OVERFLOW_ADCVAL   =-4096
 BMM150_ZAXIS_HALL_OVERFLOW_ADCVAL    =-16384
 BMM150_OVERFLOW_OUTPUT               =-32768
 BMM150_NEGATIVE_SATURATION_Z         =-32767
 BMM150_POSITIVE_SATURATION_Z         =32767
 BMM150_OVERFLOW_OUTPUT_FLOAT         =0.0

/**\name PRESET MODE DEFINITIONS  */
 BMM150_PRESETMODE_LOWPOWER           =0x01
 BMM150_PRESETMODE_REGULAR            =0x02
 BMM150_PRESETMODE_HIGHACCURACY       =0x03
 BMM150_PRESETMODE_ENHANCED           =0x04

/**\name PRESET MODES - REPETITIONS-XY RATES */
 BMM150_LOWPOWER_REPXY                =1
 BMM150_REGULAR_REPXY                 =4
 BMM150_ENHANCED_REPXY                =7
 BMM150_HIGHACCURACY_REPXY            =23

/**\name PRESET MODES - REPETITIONS-Z RATES */
 BMM150_LOWPOWER_REPZ                 =2
 BMM150_REGULAR_REPZ                  =14
 BMM150_ENHANCED_REPZ                 =26
 BMM150_HIGHACCURACY_REPZ             =82

/**\name Register read lengths  */
 BMM150_SELF_TEST_LEN                 =5
 BMM150_SETTING_DATA_LEN              =8
 BMM150_XYZR_DATA_LEN                 =8

/**\name Self test selection macros */
 BMM150_NORMAL_SELF_TEST              =0
 BMM150_ADVANCED_SELF_TEST            =1

/**\name Self test settings */
 BMM150_DISABLE_XY_AXIS               =0x03
 BMM150_SELF_TEST_REP_Z               =0x04

/**\name Advanced self-test current settings */
 BMM150_DISABLE_SELF_TEST_CURRENT     =0x00
 BMM150_ENABLE_NEGATIVE_CURRENT       =0x02
 BMM150_ENABLE_POSITIVE_CURRENT       =0x03

/**\name Normal self-test status */
 BMM150_SELF_TEST_STATUS_XYZ_FAIL     =0x00
 BMM150_SELF_TEST_STATUS_SUCCESS      =0x07

/** Mask definitions */
 BMI160_ACCEL_BW_MASK                 =0x70
 BMI160_ACCEL_ODR_MASK                =0x0F
 BMI160_ACCEL_UNDERSAMPLING_MASK      =0x80
 BMI160_ACCEL_RANGE_MASK              =0x0F
 BMI160_GYRO_BW_MASK                  =0x30
 BMI160_GYRO_ODR_MASK                 =0x0F
 BMI160_GYRO_RANGE_MSK                =0x07

/** Mask definitions for INT_EN registers */
 BMI160_ANY_MOTION_X_INT_EN_MASK      =0x01
 BMI160_HIGH_G_X_INT_EN_MASK          =0x01
 BMI160_NO_MOTION_X_INT_EN_MASK       =0x01
 BMI160_ANY_MOTION_Y_INT_EN_MASK      =0x02
 BMI160_HIGH_G_Y_INT_EN_MASK          =0x02
 BMI160_NO_MOTION_Y_INT_EN_MASK       =0x02
 BMI160_ANY_MOTION_Z_INT_EN_MASK      =0x04
 BMI160_HIGH_G_Z_INT_EN_MASK          =0x04
 BMI160_NO_MOTION_Z_INT_EN_MASK       =0x04
 BMI160_SIG_MOTION_INT_EN_MASK        =0x07
 BMI160_ANY_MOTION_ALL_INT_EN_MASK    =0x07
 BMI160_STEP_DETECT_INT_EN_MASK       =0x08
 BMI160_DOUBLE_TAP_INT_EN_MASK        =0x10
 BMI160_SINGLE_TAP_INT_EN_MASK        =0x20
 BMI160_FIFO_FULL_INT_EN_MASK         =0x20
 BMI160_ORIENT_INT_EN_MASK            =0x40
 BMI160_FIFO_WATERMARK_INT_EN_MASK    =0x40
 BMI160_LOW_G_INT_EN_MASK             =0x08
 BMI160_STEP_DETECT_EN_MASK           =0x08
 BMI160_FLAT_INT_EN_MASK              =0x80
 BMI160_DATA_RDY_INT_EN_MASK          =0x10

/** PMU status Macros */
 BMI160_AUX_PMU_SUSPEND               =0x00
 BMI160_AUX_PMU_NORMAL                =0x01
 BMI160_AUX_PMU_LOW_POWER             =0x02

 BMI160_GYRO_PMU_SUSPEND              =0x00
 BMI160_GYRO_PMU_NORMAL               =0x01
 BMI160_GYRO_PMU_FSU                  =0x03

 BMI160_ACCEL_PMU_SUSPEND             =0x00
 BMI160_ACCEL_PMU_NORMAL              =0x01
 BMI160_ACCEL_PMU_LOW_POWER           =0x02

/** Mask definitions for INT_OUT_CTRL register */
 BMI160_INT1_EDGE_CTRL_MASK           =0x01
 BMI160_INT1_OUTPUT_MODE_MASK         =0x04
 BMI160_INT1_OUTPUT_TYPE_MASK         =0x02
 BMI160_INT1_OUTPUT_EN_MASK           =0x08
 BMI160_INT2_EDGE_CTRL_MASK           =0x10
 BMI160_INT2_OUTPUT_MODE_MASK         =0x40
 BMI160_INT2_OUTPUT_TYPE_MASK         =0x20
 BMI160_INT2_OUTPUT_EN_MASK           =0x80

/** Mask definitions for INT_LATCH register */
 BMI160_INT1_INPUT_EN_MASK            =0x10
 BMI160_INT2_INPUT_EN_MASK            =0x20
 BMI160_INT_LATCH_MASK                =0x0F

/** Mask definitions for INT_MAP register */
 BMI160_INT1_LOW_G_MASK               =0x01
 BMI160_INT1_HIGH_G_MASK              =0x02
 BMI160_INT1_SLOPE_MASK               =0x04
 BMI160_INT1_NO_MOTION_MASK           =0x08
 BMI160_INT1_DOUBLE_TAP_MASK          =0x10
 BMI160_INT1_SINGLE_TAP_MASK          =0x20
 BMI160_INT1_FIFO_FULL_MASK           =0x20
 BMI160_INT1_FIFO_WM_MASK             =0x40
 BMI160_INT1_ORIENT_MASK              =0x40
 BMI160_INT1_FLAT_MASK                =0x80
 BMI160_INT1_DATA_READY_MASK          =0x80
 BMI160_INT2_LOW_G_MASK               =0x01
 BMI160_INT1_LOW_STEP_DETECT_MASK     =0x01
 BMI160_INT2_LOW_STEP_DETECT_MASK     =0x01
 BMI160_INT2_HIGH_G_MASK              =0x02
 BMI160_INT2_FIFO_FULL_MASK           =0x02
 BMI160_INT2_FIFO_WM_MASK             =0x04
 BMI160_INT2_SLOPE_MASK               =0x04
 BMI160_INT2_DATA_READY_MASK          =0x08
 BMI160_INT2_NO_MOTION_MASK           =0x08
 BMI160_INT2_DOUBLE_TAP_MASK          =0x10
 BMI160_INT2_SINGLE_TAP_MASK          =0x20
 BMI160_INT2_ORIENT_MASK              =0x40
 BMI160_INT2_FLAT_MASK                =0x80

/** Mask definitions for INT_DATA register */
 BMI160_TAP_SRC_INT_MASK              =0x08
 BMI160_LOW_HIGH_SRC_INT_MASK         =0x80
 BMI160_MOTION_SRC_INT_MASK           =0x80

/** Mask definitions for INT_MOTION register */
 BMI160_SLOPE_INT_DUR_MASK            =0x03
 BMI160_NO_MOTION_INT_DUR_MASK        =0xFC
 BMI160_NO_MOTION_SEL_BIT_MASK        =0x01

/** Mask definitions for INT_TAP register */
 BMI160_TAP_DUR_MASK                  =0x07
 BMI160_TAP_SHOCK_DUR_MASK            =0x40
 BMI160_TAP_QUIET_DUR_MASK            =0x80
 BMI160_TAP_THRES_MASK                =0x1F

/** Mask definitions for INT_FLAT register */
 BMI160_FLAT_THRES_MASK               =0x3F
 BMI160_FLAT_HOLD_TIME_MASK           =0x30
 BMI160_FLAT_HYST_MASK                =0x07

/** Mask definitions for INT_LOWHIGH register */
 BMI160_LOW_G_HYST_MASK               =0x03
 BMI160_LOW_G_LOW_MODE_MASK           =0x04
 BMI160_HIGH_G_HYST_MASK              =0xC0

/** Mask definitions for INT_SIG_MOTION register */
 BMI160_SIG_MOTION_SEL_MASK           =0x02
 BMI160_SIG_MOTION_SKIP_MASK          =0x0C
 BMI160_SIG_MOTION_PROOF_MASK         =0x30

/** Mask definitions for INT_ORIENT register */
 BMI160_ORIENT_MODE_MASK              =0x03
 BMI160_ORIENT_BLOCK_MASK             =0x0C
 BMI160_ORIENT_HYST_MASK              =0xF0
 BMI160_ORIENT_THETA_MASK             =0x3F
 BMI160_ORIENT_UD_ENABLE              =0x40
 BMI160_AXES_EN_MASK                  =0x80

/** Mask definitions for FIFO_CONFIG register */
 BMI160_FIFO_GYRO                     =0x80
 BMI160_FIFO_ACCEL                    =0x40
 BMI160_FIFO_AUX                      =0x20
 BMI160_FIFO_TAG_INT1                 =0x08
 BMI160_FIFO_TAG_INT2                 =0x04
 BMI160_FIFO_TIME                     =0x02
 BMI160_FIFO_HEADER                   =0x10
 BMI160_FIFO_CONFIG_1_MASK            =0xFE

/** Mask definitions for STEP_CONF register */
 BMI160_STEP_COUNT_EN_BIT_MASK        =0x08
 BMI160_STEP_DETECT_MIN_THRES_MASK    =0x18
 BMI160_STEP_DETECT_STEPTIME_MIN_MASK =0x07
 BMI160_STEP_MIN_BUF_MASK             =0x07

/** Mask definition for FIFO Header Data Tag */
 BMI160_FIFO_TAG_INTR_MASK            =0xFC

/** Fifo byte counter mask definitions */
 BMI160_FIFO_BYTE_COUNTER_MASK        =0x07

/** Enable/disable bit value */
 BMI160_ENABLE                        =0x01
 BMI160_DISABLE                       =0x00

/** Latch Duration */
 BMI160_LATCH_DUR_NONE                =0x00
 BMI160_LATCH_DUR_312_5_MICRO_SEC     =0x01
 BMI160_LATCH_DUR_625_MICRO_SEC       =0x02
 BMI160_LATCH_DUR_1_25_MILLI_SEC      =0x03
 BMI160_LATCH_DUR_2_5_MILLI_SEC       =0x04
 BMI160_LATCH_DUR_5_MILLI_SEC         =0x05
 BMI160_LATCH_DUR_10_MILLI_SEC        =0x06
 BMI160_LATCH_DUR_20_MILLI_SEC        =0x07
 BMI160_LATCH_DUR_40_MILLI_SEC        =0x08
 BMI160_LATCH_DUR_80_MILLI_SEC        =0x09
 BMI160_LATCH_DUR_160_MILLI_SEC       =0x0A
 BMI160_LATCH_DUR_320_MILLI_SEC       =0x0B
 BMI160_LATCH_DUR_640_MILLI_SEC       =0x0C
 BMI160_LATCH_DUR_1_28_SEC            =0x0D
 BMI160_LATCH_DUR_2_56_SEC            =0x0E
 BMI160_LATCHED                       =0x0F

/** BMI160 Register map */
 BMI160_CHIP_ID_ADDR                  =0x00
 BMI160_ERROR_REG_ADDR                =0x02
 BMI160_PMU_STATUS_ADDR               =0x03
 BMI160_AUX_DATA_ADDR                 =0x04
 BMI160_GYRO_DATA_ADDR                =0x0C
 BMI160_ACCEL_DATA_ADDR               =0x12
 BMI160_STATUS_ADDR                   =0x1B
 BMI160_INT_STATUS_ADDR               =0x1C
 BMI160_FIFO_LENGTH_ADDR              =0x22
 BMI160_FIFO_DATA_ADDR                =0x24
 BMI160_ACCEL_CONFIG_ADDR             =0x40
 BMI160_ACCEL_RANGE_ADDR              =0x41
 BMI160_GYRO_CONFIG_ADDR              =0x42
 BMI160_GYRO_RANGE_ADDR               =0x43
 BMI160_AUX_ODR_ADDR                  =0x44
 BMI160_FIFO_DOWN_ADDR                =0x45
 BMI160_FIFO_CONFIG_0_ADDR            =0x46
 BMI160_FIFO_CONFIG_1_ADDR            =0x47
 BMI160_AUX_IF_0_ADDR                 =0x4B
 BMI160_AUX_IF_1_ADDR                 =0x4C
 BMI160_AUX_IF_2_ADDR                 =0x4D
 BMI160_AUX_IF_3_ADDR                 =0x4E
 BMI160_AUX_IF_4_ADDR                 =0x4F
 BMI160_INT_ENABLE_0_ADDR             =0x50
 BMI160_INT_ENABLE_1_ADDR             =0x51
 BMI160_INT_ENABLE_2_ADDR             =0x52
 BMI160_INT_OUT_CTRL_ADDR             =0x53
 BMI160_INT_LATCH_ADDR                =0x54
 BMI160_INT_MAP_0_ADDR                =0x55
 BMI160_INT_MAP_1_ADDR                =0x56
 BMI160_INT_MAP_2_ADDR                =0x57
 BMI160_INT_DATA_0_ADDR               =0x58
 BMI160_INT_DATA_1_ADDR               =0x59
 BMI160_INT_LOWHIGH_0_ADDR            =0x5A
 BMI160_INT_LOWHIGH_1_ADDR            =0x5B
 BMI160_INT_LOWHIGH_2_ADDR            =0x5C
 BMI160_INT_LOWHIGH_3_ADDR            =0x5D
 BMI160_INT_LOWHIGH_4_ADDR            =0x5E
 BMI160_INT_MOTION_0_ADDR             =0x5F
 BMI160_INT_MOTION_1_ADDR             =0x60
 BMI160_INT_MOTION_2_ADDR             =0x61
 BMI160_INT_MOTION_3_ADDR             =0x62
 BMI160_INT_TAP_0_ADDR                =0x63
 BMI160_INT_TAP_1_ADDR                =0x64
 BMI160_INT_ORIENT_0_ADDR             =0x65
 BMI160_INT_ORIENT_1_ADDR             =0x66
 BMI160_INT_FLAT_0_ADDR               =0x67
 BMI160_INT_FLAT_1_ADDR               =0x68
 BMI160_FOC_CONF_ADDR                 =0x69
 BMI160_CONF_ADDR                     =0x6A

 BMI160_IF_CONF_ADDR                  =0x6B
 BMI160_SELF_TEST_ADDR                =0x6D
 BMI160_OFFSET_ADDR                   =0x71
 BMI160_OFFSET_CONF_ADDR              =0x77
 BMI160_INT_STEP_CNT_0_ADDR           =0x78
 BMI160_INT_STEP_CONFIG_0_ADDR        =0x7A
 BMI160_INT_STEP_CONFIG_1_ADDR        =0x7B
 BMI160_COMMAND_REG_ADDR              =0x7E
 BMI160_SPI_COMM_TEST_ADDR            =0x7F
 BMI160_INTL_PULLUP_CONF_ADDR         =0x85

/** Error code definitions */
 BMI160_OK                            =0
 BMI160_E_NULL_PTR                    =-1
 BMI160_E_COM_FAIL                    =-2
 BMI160_E_DEV_NOT_FOUND               =-3
 BMI160_E_OUT_OF_RANGE                =-4
 BMI160_E_INVALID_INPUT               =-5
 BMI160_E_ACCEL_ODR_BW_INVALID        =-6
 BMI160_E_GYRO_ODR_BW_INVALID         =-7
 BMI160_E_LWP_PRE_FLTR_INT_INVALID    =-8
 BMI160_E_LWP_PRE_FLTR_INVALID        =-9
 BMI160_E_AUX_NOT_FOUND               =-10
 BMI160_FOC_FAILURE                   =-11
 BMI160_READ_WRITE_LENGHT_INVALID     =-12

/**\name API warning codes */
 BMI160_W_GYRO_SELF_TEST_FAIL         =1
 BMI160_W_ACCEl_SELF_TEST_FAIL        =2

/** BMI160 unique chip identifier */
 BMI160_CHIP_ID                       =0xD8

/** Soft reset command */
 BMI160_SOFT_RESET_CMD                =0xb6
 BMI160_SOFT_RESET_DELAY_MS           =1

/** Start FOC command */
 BMI160_START_FOC_CMD                 =0x03

/** NVM backup enabling command */
 BMI160_NVM_BACKUP_EN                 =0xA0

/* Delay in ms settings */
 BMI160_ACCEL_DELAY_MS                =5
 BMI160_GYRO_DELAY_MS                 =81
 BMI160_ONE_MS_DELAY                  =1
 BMI160_AUX_COM_DELAY                 =10
 BMI160_GYRO_SELF_TEST_DELAY          =20
 BMI160_ACCEL_SELF_TEST_DELAY         =50

/** Self test configurations */
 BMI160_ACCEL_SELF_TEST_CONFIG        =0x2C
 BMI160_ACCEL_SELF_TEST_POSITIVE_EN   =0x0D
 BMI160_ACCEL_SELF_TEST_NEGATIVE_EN   =0x09
 BMI160_ACCEL_SELF_TEST_LIMIT         =8192

/** Power mode settings */
/* Accel power mode */
 BMI160_ACCEL_NORMAL_MODE             =0x11
 BMI160_ACCEL_LOWPOWER_MODE           =0x12
 BMI160_ACCEL_SUSPEND_MODE            =0x10

/* Gyro power mode */
 BMI160_GYRO_SUSPEND_MODE             =0x14
 BMI160_GYRO_NORMAL_MODE              =0x15
 BMI160_GYRO_FASTSTARTUP_MODE         =0x17

/* Aux power mode */
 BMI160_AUX_SUSPEND_MODE              =0x18
 BMI160_AUX_NORMAL_MODE               =0x19
 BMI160_AUX_LOWPOWER_MODE             =0x1A

/** Range settings */
/* Accel Range */
 BMI160_ACCEL_RANGE_2G                =0x03
 BMI160_ACCEL_RANGE_4G                =0x05
 BMI160_ACCEL_RANGE_8G                =0x08
 BMI160_ACCEL_RANGE_16G               =0x0C

/* Gyro Range */
 BMI160_GYRO_RANGE_2000_DPS           =0x00
 BMI160_GYRO_RANGE_1000_DPS           =0x01
 BMI160_GYRO_RANGE_500_DPS            =0x02
 BMI160_GYRO_RANGE_250_DPS            =0x03
 BMI160_GYRO_RANGE_125_DPS            =0x04

/** Bandwidth settings */
/* Accel Bandwidth */
 BMI160_ACCEL_BW_OSR4_AVG1            =0x00
 BMI160_ACCEL_BW_OSR2_AVG2            =0x01
 BMI160_ACCEL_BW_NORMAL_AVG4          =0x02
 BMI160_ACCEL_BW_RES_AVG8             =0x03
 BMI160_ACCEL_BW_RES_AVG16            =0x04
 BMI160_ACCEL_BW_RES_AVG32            =0x05
 BMI160_ACCEL_BW_RES_AVG64            =0x06
 BMI160_ACCEL_BW_RES_AVG128           =0x07

 BMI160_GYRO_BW_OSR4_MODE             =0x00
 BMI160_GYRO_BW_OSR2_MODE             =0x01
 BMI160_GYRO_BW_NORMAL_MODE           =0x02

/* Output Data Rate settings */
/* Accel Output data rate */
 BMI160_ACCEL_ODR_RESERVED            =0x00
 BMI160_ACCEL_ODR_0_78HZ              =0x01
 BMI160_ACCEL_ODR_1_56HZ              =0x02
 BMI160_ACCEL_ODR_3_12HZ              =0x03
 BMI160_ACCEL_ODR_6_25HZ              =0x04
 BMI160_ACCEL_ODR_12_5HZ              =0x05
 BMI160_ACCEL_ODR_25HZ                =0x06
 BMI160_ACCEL_ODR_50HZ                =0x07
 BMI160_ACCEL_ODR_100HZ               =0x08
 BMI160_ACCEL_ODR_200HZ               =0x09
 BMI160_ACCEL_ODR_400HZ               =0x0A
 BMI160_ACCEL_ODR_800HZ               =0x0B
 BMI160_ACCEL_ODR_1600HZ              =0x0C
 BMI160_ACCEL_ODR_RESERVED0           =0x0D
 BMI160_ACCEL_ODR_RESERVED1           =0x0E
 BMI160_ACCEL_ODR_RESERVED2           =0x0F

/* Gyro Output data rate */
 BMI160_GYRO_ODR_RESERVED             =0x00
 BMI160_GYRO_ODR_25HZ                 =0x06
 BMI160_GYRO_ODR_50HZ                 =0x07
 BMI160_GYRO_ODR_100HZ                =0x08
 BMI160_GYRO_ODR_200HZ                =0x09
 BMI160_GYRO_ODR_400HZ                =0x0A
 BMI160_GYRO_ODR_800HZ                =0x0B
 BMI160_GYRO_ODR_1600HZ               =0x0C
 BMI160_GYRO_ODR_3200HZ               =0x0D

/* Auxiliary sensor Output data rate */
 BMI160_AUX_ODR_RESERVED              =0x00
 BMI160_AUX_ODR_0_78HZ                =0x01
 BMI160_AUX_ODR_1_56HZ                =0x02
 BMI160_AUX_ODR_3_12HZ                =0x03
 BMI160_AUX_ODR_6_25HZ                =0x04
 BMI160_AUX_ODR_12_5HZ                =0x05
 BMI160_AUX_ODR_25HZ                  =0x06
 BMI160_AUX_ODR_50HZ                  =0x07
 BMI160_AUX_ODR_100HZ                 =0x08
 BMI160_AUX_ODR_200HZ                 =0x09
 BMI160_AUX_ODR_400HZ                 =0x0A
 BMI160_AUX_ODR_800HZ                 =0x0B

/* Maximum limits definition */
 BMI160_ACCEL_ODR_MAX                 =15
 BMI160_ACCEL_BW_MAX                  =2
 BMI160_ACCEL_RANGE_MAX               =12
 BMI160_GYRO_ODR_MAX                  =13
 BMI160_GYRO_BW_MAX                   =2
 BMI160_GYRO_RANGE_MAX                =4

/** FIFO_CONFIG Definitions */
 BMI160_FIFO_TIME_ENABLE              =0x02
 BMI160_FIFO_TAG_INT2_ENABLE          =0x04
 BMI160_FIFO_TAG_INT1_ENABLE          =0x08
 BMI160_FIFO_HEAD_ENABLE              =0x10
 BMI160_FIFO_M_ENABLE                 =0x20
 BMI160_FIFO_A_ENABLE                 =0x40
 BMI160_FIFO_M_A_ENABLE               =0x60
 BMI160_FIFO_G_ENABLE                 =0x80
 BMI160_FIFO_M_G_ENABLE               =0xA0
 BMI160_FIFO_G_A_ENABLE               =0xC0
 BMI160_FIFO_M_G_A_ENABLE             =0xE0

/* Macro to specify the number of bytes over-read from the
 * FIFO in order to get the sensor time at the end of FIFO */
 BMI160_FIFO_BYTES_OVERREAD           =25

/* Accel, gyro and aux. sensor length and also their combined
 * length definitions in FIFO */
 BMI160_FIFO_G_LENGTH                 =6
 BMI160_FIFO_A_LENGTH                 =6
 BMI160_FIFO_M_LENGTH                 =8
 BMI160_FIFO_GA_LENGTH                =12
 BMI160_FIFO_MA_LENGTH                =14
 BMI160_FIFO_MG_LENGTH                =14
 BMI160_FIFO_MGA_LENGTH               =20

/** FIFO Header Data definitions */
 BMI160_FIFO_HEAD_SKIP_FRAME          =0x40
 BMI160_FIFO_HEAD_SENSOR_TIME         =0x44
 BMI160_FIFO_HEAD_INPUT_CONFIG        =0x48
 BMI160_FIFO_HEAD_OVER_READ           =0x80
 BMI160_FIFO_HEAD_A                   =0x84
 BMI160_FIFO_HEAD_G                   =0x88
 BMI160_FIFO_HEAD_G_A                 =0x8C
 BMI160_FIFO_HEAD_M                   =0x90
 BMI160_FIFO_HEAD_M_A                 =0x94
 BMI160_FIFO_HEAD_M_G                 =0x98
 BMI160_FIFO_HEAD_M_G_A               =0x9C

/** FIFO sensor time length definitions */
 BMI160_SENSOR_TIME_LENGTH            =3

/** FIFO DOWN selection */
/* Accel fifo down-sampling values*/
  BMI160_ACCEL_FIFO_DOWN_ZERO         =0x00
  BMI160_ACCEL_FIFO_DOWN_ONE          =0x10
  BMI160_ACCEL_FIFO_DOWN_TWO          =0x20
  BMI160_ACCEL_FIFO_DOWN_THREE        =0x30
  BMI160_ACCEL_FIFO_DOWN_FOUR         =0x40
  BMI160_ACCEL_FIFO_DOWN_FIVE         =0x50
  BMI160_ACCEL_FIFO_DOWN_SIX          =0x60
  BMI160_ACCEL_FIFO_DOWN_SEVEN        =0x70

/* Gyro fifo down-smapling values*/
  BMI160_GYRO_FIFO_DOWN_ZERO          =0x00
  BMI160_GYRO_FIFO_DOWN_ONE           =0x01
  BMI160_GYRO_FIFO_DOWN_TWO           =0x02
  BMI160_GYRO_FIFO_DOWN_THREE         =0x03
  BMI160_GYRO_FIFO_DOWN_FOUR          =0x04
  BMI160_GYRO_FIFO_DOWN_FIVE          =0x05
  BMI160_GYRO_FIFO_DOWN_SIX           =0x06
  BMI160_GYRO_FIFO_DOWN_SEVEN         =0x07

/* Accel Fifo filter enable*/
  BMI160_ACCEL_FIFO_FILT_EN           =0x80

/* Gyro Fifo filter enable*/
  BMI160_GYRO_FIFO_FILT_EN            =0x08

/** Definitions to check validity of FIFO frames */
 FIFO_CONFIG_MSB_CHECK                =0x80
 FIFO_CONFIG_LSB_CHECK                =0x00

/*! BMI160 accel FOC configurations */
 BMI160_FOC_ACCEL_DISABLED            =0x00
 BMI160_FOC_ACCEL_POSITIVE_G          =0x01
 BMI160_FOC_ACCEL_NEGATIVE_G          =0x02
 BMI160_FOC_ACCEL_0G                  =0x03

/** Array Parameter DefinItions */
 BMI160_SENSOR_TIME_LSB_BYTE          =0
 BMI160_SENSOR_TIME_XLSB_BYTE         =1
 BMI160_SENSOR_TIME_MSB_BYTE          =2

/** Interface settings */
 BMI160_SPI_INTF                      =1
 BMI160_I2C_INTF                      =0
 BMI160_SPI_RD_MASK                   =0x80
 BMI160_SPI_WR_MASK                   =0x7F

/* Sensor & time select definition*/
 BMI160_ACCEL_SEL                     =0x01
 BMI160_GYRO_SEL                      =0x02
 BMI160_TIME_SEL                      =0x04

/* Sensor select mask*/
 BMI160_SEN_SEL_MASK                  =0x07

/* Error code mask */
 BMI160_ERR_REG_MASK                  =0x0F

/* BMI160 I2C address */
 BMI160_I2C_ADDR                      =0x68

/* BMI160 secondary IF address */
 BMI160_AUX_BMM150_I2C_ADDR           =0x10

/** BMI160 Length definitions */
 BMI160_ONE                           =1
 BMI160_TWO                           =2
 BMI160_THREE                         =3
 BMI160_FOUR                          =4
 BMI160_FIVE                          =5

/** BMI160 fifo level Margin */
 BMI160_FIFO_LEVEL_MARGIN             =16

/** BMI160 fifo flush Command */
 BMI160_FIFO_FLUSH_VALUE              =0xB0

/** BMI160 offset values for xyz axes of accel */
 BMI160_ACCEL_MIN_OFFSET              =-128
 BMI160_ACCEL_MAX_OFFSET              =127

/** BMI160 offset values for xyz axes of gyro */
 BMI160_GYRO_MIN_OFFSET               =-512
 BMI160_GYRO_MAX_OFFSET               =511

/** BMI160 fifo full interrupt position and mask */
 BMI160_FIFO_FULL_INT_POS             =5
 BMI160_FIFO_FULL_INT_MSK             =0x20
 BMI160_FIFO_WTM_INT_POS              =6
 BMI160_FIFO_WTM_INT_MSK              =0x40

 BMI160_FIFO_FULL_INT_PIN1_POS        =5
 BMI160_FIFO_FULL_INT_PIN1_MSK        =0x20
 BMI160_FIFO_FULL_INT_PIN2_POS        =1
 BMI160_FIFO_FULL_INT_PIN2_MSK        =0x02

 BMI160_FIFO_WTM_INT_PIN1_POS         =6
 BMI160_FIFO_WTM_INT_PIN1_MSK         =0x40
 BMI160_FIFO_WTM_INT_PIN2_POS         =2
 BMI160_FIFO_WTM_INT_PIN2_MSK         =0x04

 BMI160_MANUAL_MODE_EN_POS            =7
 BMI160_MANUAL_MODE_EN_MSK            =0x80
 BMI160_AUX_READ_BURST_POS            =0
 BMI160_AUX_READ_BURST_MSK            =0x03

 BMI160_GYRO_SELF_TEST_POS            =4
 BMI160_GYRO_SELF_TEST_MSK            =0x10
 BMI160_GYRO_SELF_TEST_STATUS_POS     =1
 BMI160_GYRO_SELF_TEST_STATUS_MSK     =0x02

 BMI160_GYRO_FOC_EN_POS               =6
 BMI160_GYRO_FOC_EN_MSK               =0x40

 BMI160_ACCEL_FOC_X_CONF_POS          =4
 BMI160_ACCEL_FOC_X_CONF_MSK          =0x30

 BMI160_ACCEL_FOC_Y_CONF_POS          =2
 BMI160_ACCEL_FOC_Y_CONF_MSK          =0x0C

 BMI160_ACCEL_FOC_Z_CONF_MSK          =0x03

 BMI160_FOC_STATUS_POS                =3
 BMI160_FOC_STATUS_MSK                =0x08

 BMI160_GYRO_OFFSET_X_MSK             =0x03

 BMI160_GYRO_OFFSET_Y_POS             =2
 BMI160_GYRO_OFFSET_Y_MSK             =0x0C

 BMI160_GYRO_OFFSET_Z_POS             =4
 BMI160_GYRO_OFFSET_Z_MSK             =0x30

 BMI160_GYRO_OFFSET_EN_POS            =7
 BMI160_GYRO_OFFSET_EN_MSK            =0x80

 BMI160_ACCEL_OFFSET_EN_POS           =6
 BMI160_ACCEL_OFFSET_EN_MSK           =0x40

 BMI160_GYRO_OFFSET_POS               =8
 BMI160_GYRO_OFFSET_MSK               =0x0300

 BMI160_NVM_UPDATE_POS                =1
 BMI160_NVM_UPDATE_MSK                =0x02

 BMI160_NVM_STATUS_POS                =4
 BMI160_NVM_STATUS_MSK                =0x10

 BMI160_MAG_POWER_MODE_MSK            =0x03

 BMI160_ACCEL_POWER_MODE_MSK          =0x30
 BMI160_ACCEL_POWER_MODE_POS          =4

 BMI160_GYRO_POWER_MODE_MSK           =0x0C
 BMI160_GYRO_POWER_MODE_POS           =2

)
