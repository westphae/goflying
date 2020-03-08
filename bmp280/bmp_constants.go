package bmp280

const (
	// I2C Address Definitions
	Address1 = 0x76
	Address2 = 0x77
	// Chip ID Definitions
	ChipID1 = 0x56
	ChipID2 = 0x57
	ChipID3 = 0x58
	// Power Mode Definitions
	SleepMode     = 0x00
	ForcedMode    = 0x01
	NormalMode    = 0x03
	SoftResetCode = 0xB6
	// Standby Time Definitions
	StandbyTime1ms    = 0x00
	StandbyTime63ms   = 0x01
	StandbyTime125ms  = 0x02
	StandbyTime250ms  = 0x03
	StandbyTime500ms  = 0x04
	StandbyTime1000ms = 0x05
	StandbyTime2000ms = 0x06
	StandbyTime4000ms = 0x07
	// Filter Definitions
	FilterCoeffOff = 0x00
	FilterCoeff2   = 0x01
	FilterCoeff4   = 0x02
	FilterCoeff8   = 0x03
	FilterCoeff16  = 0x04
	// Oversampling Definitions
	OversampSkipped = 0x00
	Oversamp1x      = 0x01
	Oversamp2x      = 0x02
	Oversamp4x      = 0x03
	Oversamp8x      = 0x04
	Oversamp16x     = 0x05
	// Working Mode Definitions
	UltraLowPowerMode       = 0x00
	LowPowerMode            = 0x01
	StandardResolutionMode  = 0x02
	HighResolutionMode      = 0x03
	UltraHighResolutionMode = 0x04

	// BMP280 registers
	RegisterCompData      = 0x88
	RegisterChipID        = 0xD0
	RegisterSoftReset     = 0xE0
	RegisterStatus        = 0xF3
	RegisterControl       = 0xF4
	RegisterConfig        = 0xF5
	RegisterPressDataMSB  = 0xF7
	RegisterPressDataLSB  = 0xF8
	RegisterPressDataXLSB = 0xF9
	RegisterTempDataMSB   = 0xFA
	RegisterTempDataLSB   = 0xFB
	RegisterTempDataXLSB  = 0xFC

	// Connection retries definition
	ConnAttempts = 5

	// Sea level reference pressure in hPa
	QNH = 1013.25

	// Buffer size for reading data from BMP
	BufSize = 256
)
