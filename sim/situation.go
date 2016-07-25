package main

import "github.com/westphae/goflying/ahrs"

type Situation interface {
	BeginTime() (float64)
	Interpolate(t float64, s *ahrs.State) (err error)
	Control(t float64, c *ahrs.Control, gyroNoise, accelNoise float64, gyroBias, accelBias []float64 ) (err error)
	Measurement(t float64, m *ahrs.Measurement, wValid, uValid, mValid bool, wn, un, mn, ab float64, mb []float64) (err error)
}
