package main

import "../ahrs"

type Situation interface {
	BeginTime() float64
	NextTime() (err error)
	UpdateState(s *ahrs.State, aBias, bBias, mBias []float64) (err error)
	UpdateMeasurement(m *ahrs.Measurement,
		uValid, wValid, sValid, mValid bool,
		uNoise, wNoise, aNoise, bNoise, mNoise float64,
		uBias, aBias, bBias, mBias []float64,
	) (err error)
	GetLogMap() (p map[string]interface{})
}
