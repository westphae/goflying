package magkal

import (
	"../ahrs"
)

type TrivialMagKalState struct {
	MagKalState
}

//NewTrivialMagKal returns a new MagKal object that just maintains a constant K,L.
func NewTrivialMagKal() (s *TrivialMagKalState) {
	s = new(TrivialMagKalState)

	s.Reset()
	s.updateLogMap(ahrs.NewMeasurement(), s.logMap)
	return
}

// Compute performs the MagKalTrivial calculations for the trivial calibration procedure.
// The Trivial procedure is just maintaining constant K, L.
// This is mainly used for testing the integration with Stratux.
func (s *TrivialMagKalState) Compute(m *ahrs.Measurement) {
	s.T = m.T

	// K, L are unaltered for the trivial MagKal.

	s.updateLogMap(m, s.logMap)
}
