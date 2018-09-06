package magkal

import (
	"../ahrs"
	"math"
)

const (
	Pi          = math.Pi
	Small       = 1e-9
	Big         = 1e9
	Deg         = Pi / 180
	AvgMagField = 4390
)

// MagKalProvider defines an AHRS (Kalman or other) algorithm, such as ahrs_kalman, ahrs_simple, etc.
type MagKalProvider interface {
	// Compute runs both the "predict" and "update" stages of the algorithm, for convenience.
	Compute(m *ahrs.Measurement)
	// SetCalibrations sets the mag scaling to k and mag offset to l.
	SetCalibrations(k, l *[3]float64)
	// GetCalibrations returns the mag scaling k and mag offset l.
	GetCalibrations() (k, l *[3]float64)
	// SetConfig allows for configuration of MagKal to be set on the fly, mainly for developers.
	SetConfig(configMap map[string]float64)
	// Valid returns whether the current state is a valid estimate or if something went wrong in the calculation.
	Valid() bool
	// Reset restarts the algorithm from scratch.
	Reset()
	// GetLogMap returns a map customized for each MagKalProvider algorithm to provide more detailed information
	// for debugging and logging.
	GetLogMap() map[string]interface{}
}

type MagKalState struct {
	K1, K2, K3 float64 // Scaling factor for magnetometer
	L1, L2, L3 float64 // Offset for magnetometer

	T float64 // Time when state last updated

	logMap map[string]interface{} // Map only for analysis/debugging
}

// SetCalibrations sets the mag scaling to k and mag offset to l.
func (s *MagKalState) SetCalibrations(k, l *[3]float64) {
	if k != nil {
		kNorm := math.Sqrt(k[0]*k[0] + k[1]*k[1] + k[2]*k[2])
		if kNorm > 0.5 {
			s.K1 = k[0]
			s.K2 = k[1]
			s.K3 = k[2]
		}
	}
	if l != nil {
		s.L1 = l[0]
		s.L2 = l[1]
		s.L3 = l[2]
	}
}

// GetCalibrations returns the mag scaling k and mag offset l.
func (s *MagKalState) GetCalibrations() (k, l *[3]float64) {
	return &[3]float64{s.K1, s.K2, s.K3}, &[3]float64{s.L1, s.L2, s.L3}
}

// SetConfig lets the user alter some of the configuration settings.
func (s *MagKalState) SetConfig(configMap map[string]float64) {
	return
}

// Valid returns whether the current state is a valid estimate or if something went wrong in the calculation.
func (s *MagKalState) Valid() (ok bool) {
	return true
}

// Reset restarts the algorithm from scratch.
func (s *MagKalState) Reset() {
	s.K1, s.K2, s.K3 = Big, Big, Big
	s.L1, s.L2, s.L3 = 0, 0, 0

	s.T = 0 // Time when state last updated

	s.logMap = make(map[string]interface{})
}

// GetLogMap returns a map providing current state and measurement values for analysis
func (s *MagKalState) GetLogMap() (p map[string]interface{}) {
	return s.logMap
}

func (s *MagKalState) updateLogMap(m *ahrs.Measurement, p map[string]interface{}) {
	var logMap = map[string]func(s *MagKalState, m *ahrs.Measurement) float64{
		"Ta": func(s *MagKalState, m *ahrs.Measurement) float64 { return s.T },
		"WValid": func(s *MagKalState, m *ahrs.Measurement) float64 {
			if m.WValid {
				return 1
			}
			return 0
		},
		"T":  func(s *MagKalState, m *ahrs.Measurement) float64 { return m.T },
		"TW": func(s *MagKalState, m *ahrs.Measurement) float64 { return m.TW },
		"W1": func(s *MagKalState, m *ahrs.Measurement) float64 { return m.W1 },
		"W2": func(s *MagKalState, m *ahrs.Measurement) float64 { return m.W2 },
		"W3": func(s *MagKalState, m *ahrs.Measurement) float64 { return m.W3 },
		"A1": func(s *MagKalState, m *ahrs.Measurement) float64 { return m.A1 },
		"A2": func(s *MagKalState, m *ahrs.Measurement) float64 { return m.A2 },
		"A3": func(s *MagKalState, m *ahrs.Measurement) float64 { return m.A3 },
		"B1": func(s *MagKalState, m *ahrs.Measurement) float64 { return m.B1 },
		"B2": func(s *MagKalState, m *ahrs.Measurement) float64 { return m.B2 },
		"B3": func(s *MagKalState, m *ahrs.Measurement) float64 { return m.B3 },
		"M1": func(s *MagKalState, m *ahrs.Measurement) float64 { return m.M1 },
		"M2": func(s *MagKalState, m *ahrs.Measurement) float64 { return m.M2 },
		"M3": func(s *MagKalState, m *ahrs.Measurement) float64 { return m.M3 },
		"K1": func(s *MagKalState, m *ahrs.Measurement) float64 { return s.K1 },
		"K2": func(s *MagKalState, m *ahrs.Measurement) float64 { return s.K2 },
		"K3": func(s *MagKalState, m *ahrs.Measurement) float64 { return s.K3 },
		"L1": func(s *MagKalState, m *ahrs.Measurement) float64 { return s.L1 },
		"L2": func(s *MagKalState, m *ahrs.Measurement) float64 { return s.L2 },
		"L3": func(s *MagKalState, m *ahrs.Measurement) float64 { return s.L3 },
	}

	for k := range logMap {
		p[k] = logMap[k](s, m)
	}
}

// NormDiff calculates the norm of the diff of two 3-vectors to see how different they are.
func NormDiff(v1, v2 *[3]float64) (res float64) {
	for i := 0; i < 3; i++ {
		res += (v1[i] - v2[i]) * (v1[i] - v2[i])
	}
	res = math.Sqrt(res)
	return
}
