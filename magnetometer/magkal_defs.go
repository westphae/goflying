package magkal

import (
	"math"

	"../ahrs"
)

const (
	Pi          = math.Pi
	Small       = 1e-9
	Big         = 1e9
	Deg         = Pi / 180
	AvgMagField = 4390
)


type MagKalState struct {
	T float64 // Time when state last updated
	K [3]float64 // Scaling factor for magnetometer
	L [3]float64 // Offset for magnetometer
	LogMap map[string]interface{} // Map only for analysis/debugging
}


// NewMagKal returns a new MagKal object that runs the algorithm passed to it.
// It is initialized with the starting K, L.
func NewMagKal(k, l [3]float64, f func(MagKalState, chan ahrs.Measurement, chan MagKalState)) (cIn chan ahrs.Measurement, cOut chan MagKalState) {
	cIn = make(chan ahrs.Measurement)
	cOut = make(chan MagKalState)
	s := MagKalState{K: k, L: l, LogMap: make(map[string]interface{})}
	s.updateLogMap(ahrs.NewMeasurement(), s.LogMap)

	go f(s, cIn, cOut)

	return
}


func (s *MagKalState) updateLogMap(m *ahrs.Measurement, p map[string]interface{}) {
	var logMapFunc = map[string]func(s *MagKalState, m *ahrs.Measurement) float64{
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
		"K1": func(s *MagKalState, m *ahrs.Measurement) float64 { return s.K[0] },
		"K2": func(s *MagKalState, m *ahrs.Measurement) float64 { return s.K[1] },
		"K3": func(s *MagKalState, m *ahrs.Measurement) float64 { return s.K[2] },
		"L1": func(s *MagKalState, m *ahrs.Measurement) float64 { return s.L[0] },
		"L2": func(s *MagKalState, m *ahrs.Measurement) float64 { return s.L[1] },
		"L3": func(s *MagKalState, m *ahrs.Measurement) float64 { return s.L[2] },
	}

	for k := range logMapFunc {
		p[k] = logMapFunc[k](s, m)
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

// NormVec calculates the norm of a 3-vector.
func NormVec(v1 [3]float64) (res float64) {
	for i := 0; i < 3; i++ {
		res += v1[i] * v1[i]
	}
	res = math.Sqrt(res)
	return
}
