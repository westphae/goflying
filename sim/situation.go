package main

import "../ahrs"

type Situation interface {
	BeginTime() float64
	Interpolate(t float64, s *ahrs.State, aBias, bBias, mBias []float64) (err error)
	Measurement(t float64, m *ahrs.Measurement,
		uValid, wValid, sValid, mValid bool,
		uNoise, wNoise, aNoise, bNoise, mNoise float64,
		uBias, aBias, bBias, mBias []float64,
	) (err error)
}
