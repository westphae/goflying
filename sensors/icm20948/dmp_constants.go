package icm20948

// I²C addresses the ICM-20948 can be strapped to. The kernel driver binds
// whichever one is named in the device-tree overlay (or in the per-session
// `echo icm20948 0x68 > /sys/bus/i2c/devices/i2c-1/new_device`); these
// constants are informational for callers that still pass one through.
const (
	MPU_ADDRESS1 = 0x68
	MPU_ADDRESS2 = 0x69
)
