package bmp280

// I²C address selectors. The kernel manages the bus; these are passed to
// NewBMP280 only to disambiguate when two BMP280s are present.
const (
	Address1 byte = 0x76
	Address2 byte = 0x77
)

// Oversampling levels. decodeOversamp in bmp280.go maps these to the integer
// ratios (1, 2, 4, 8, 16) that the IIO driver accepts via
// in_<channel>_oversampling_ratio. OversampSkipped is treated as "leave the
// kernel default in place".
const (
	OversampSkipped byte = 0x00
	Oversamp1x      byte = 0x01
	Oversamp2x      byte = 0x02
	Oversamp4x      byte = 0x03
	Oversamp8x      byte = 0x04
	Oversamp16x     byte = 0x05
)
