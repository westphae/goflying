package magkal

import (
	"math"

	"../ahrs"
)

type SimpleMagKalState struct {
	MagKalState
	m1Min, m1Max float64
	m2Min, m2Max float64
	m3Min, m3Max float64
}

//NewSimpleMagKal returns a new MagKal object that just maintains a constant K,L.
func NewSimpleMagKal() (s *SimpleMagKalState) {
	s = new(SimpleMagKalState)
	s.Reset()

	s.m1Min, s.m1Max = Big, -Big
	s.m2Min, s.m2Max = Big, -Big
	s.m3Min, s.m3Max = Big, -Big

	s.updateLogMap(ahrs.NewMeasurement(), s.logMap)
	return
}

// Compute performs the MagKalSimple calculations for the trivial calibration procedure.
// The Simple procedure bases K, L on the measured min/max value along each axis.
// It can provide a useful calibration but requires careful manual manipulation.
func (s *SimpleMagKalState) Compute(m *ahrs.Measurement) {
	s.T = m.T

	s.m1Min, s.m1Max = math.Min(s.m1Min, m.M1), math.Max(s.m1Max, m.M1)
	s.m2Min, s.m2Max = math.Min(s.m2Min, m.M2), math.Max(s.m2Max, m.M2)
	s.m3Min, s.m3Max = math.Min(s.m3Min, m.M3), math.Max(s.m3Max, m.M3)

	if s.m1Max-s.m1Min > 2*AvgMagField/s.K1 {
		s.K1 = 2 * AvgMagField / (s.m1Max - s.m1Min)
		s.L1 = -s.K1 * (s.m1Max + s.m1Min) / 2
	}

	if s.m2Max-s.m2Min > 2*AvgMagField/s.K2 {
		s.K2 = 2 * AvgMagField / (s.m2Max - s.m2Min)
		s.L2 = -s.K2 * (s.m2Max + s.m2Min) / 2
	}

	if s.m3Max-s.m3Min > 2*AvgMagField/s.K3 {
		s.K3 = 2 * AvgMagField / (s.m3Max - s.m3Min)
		s.L3 = -s.K3 * (s.m3Max + s.m3Min) / 2
	}

	s.updateLogMap(m, s.logMap)
}
