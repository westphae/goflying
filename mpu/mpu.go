package mpu

import (
	"time"
)

const (
	bufSize  = 250 // Size of buffer storing instantaneous sensor values
	scaleMag = 9830.0 / 65536
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

// IMU represents an Inertial Measurement Unit type of chip such as an InvenSense MPU9250.
type MPUReader struct {
	 C                     <-chan *MPUData // Current instantaneous sensor values
	 CAvg                  <-chan *MPUData // Average sensor values (since CAvg last read)
	 CBuf                  <-chan *MPUData // Buffer of instantaneous sensor values
 }

type IMU interface {
	CloseMPU() // Stops the driver from reading the MPU.
	SetSampleRate(rate byte) (err error) // Changes the sampling rate of the MPU.
	SampleRate() (rate int) // Returns the current sample rate of the MPU9250, in Hz.
	SetGyroLPF(rate byte) (err error) // Sets the low pass filter for the gyro.
	SetAccelLPF(rate byte) (err error) // Sets the low pass filter for the accelerometer.
	EnableGyroBiasCal(enable bool) (err error) // Enables or disables motion bias compensation for the gyro.
	MagEnabled() (enabled bool) // Returns whether or not the magnetometer is being read.
	SetAccelSensitivity(sensitivityAccel int) (err error) // Sets the accelerometer sensitivity of the MPU
	SetGyroSensitivity(sensitivityGyro int) (err error) // Sets the gyro sensitivity of the MPU
	ReadAccelBias(sensitivityAccel int) error // Reads the bias accelerometer value stored on the chip.
	ReadGyroBias(sensitivityGyro int) error // Reads the bias gyro value stored on the chip.
	ReadMagCalibration() error // Reads the magnetometer bias values stored on the chip.
}
