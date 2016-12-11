package ahrs

import (
	"math"
	"log"
)

const (
	KSHORT = 0.1	// Decay time for short-term moving average, 1s
	KLONG  = 0.01	// Decay time for long-term moving average, 10s
)

type HeuristicState struct {
			State
	Inertial	bool
	HeadingValid    bool
	A1, A2, A3      float64

	W10, W20, W30   float64
	W1S, W2S, W3S	float64
	W1L, W2L, W3L	float64
	A1S, A2S, A3S	float64
	A1L, A2L, A3L	float64
	B1S, B2S, B3S	float64
	B1L, B2L, B3L	float64
	M1S, M2S, M3S	float64
	M1L, M2L, M3L	float64
}

// GetStateMap returns the extra non-Kalman state information
func (s *HeuristicState) GetStateMap() (dat *map[string]float64) {
	return
}

func InitializeHeuristic(m *Measurement) (s *HeuristicState) {
	s = new(HeuristicState)
	return s
}

func (s *HeuristicState) Compute(m *Measurement) {
	s.Predict(m.T)
	s.Update(m)
}

func (s *HeuristicState) Predict(t float64) {
	return
}

func (s *HeuristicState) Update(m *Measurement) {
	// Update the smoothed sensor values
	s.A1 = KSHORT * m.A1 + (1 - KSHORT) * s.A1
	s.A2 = KSHORT * m.A2 + (1 - KSHORT) * s.A2
	s.A3 = KSHORT * m.A3 + (1 - KSHORT) * s.A3

	// Update the smoothed GPS values
	if m.WValid {
		if s.W10 == 0 && s.W20 == 0 && s.W30 == 0 {
			// Startup: don't overdo acceleration
			s.W10 = m.W1
			s.W20 = m.W2
			s.W30 = m.W3
		}

		// Update moving averages
		s.W1S = KSHORT * m.W1 + (1 - KSHORT) * s.W1S
		s.W2S = KSHORT * m.W2 + (1 - KSHORT) * s.W2S
		s.W3S = KSHORT * m.W3 + (1 - KSHORT) * s.W3S

		// Compute instantaneous accelerations from GPS (Earth frame)
		// This is what makes the sensor frame non-inertial
		s.Z1 = KSHORT * (m.W1 - s.W10) / (m.T - s.T) / G + (1 - KSHORT) * s.Z1
		s.Z2 = KSHORT * (m.W2 - s.W20) / (m.T - s.T) / G + (1 - KSHORT) * s.Z2
		s.Z3 = KSHORT * (m.W3 - s.W30) / (m.T - s.T) / G + (1 - KSHORT) * s.Z3
	} else {
		s.W1S = 0
		s.W2S = 0
		s.W3S = 0
		s.Z1 = 0
		s.Z2 = 0
		s.Z3 = 0
	}

	// Now, subtract earth-frame accel from gravity to get total accel in earth frame
	ae1 := -s.Z1
	ae2 := -s.Z2
	ae3 := -s.Z3 - 1

	// We need to map this to the accel measured by the sensor
	// First, construct quaternion qea mapping from airplane into earth frame
	q0, q1, q2, q3 := QuaternionAToB(ae1, ae2, ae3, s.A1, s.A2, s.A3)

	// This is degenerate for rotations around ae, so remove that ambiguity by
	// minimizing difference between sensor orientation and GPS track if we have GPS
	// or just point north if no GPS
	var p0, p1, p2, p3 float64

	var (
		we1 = 1.0
		we2 = 0.0
		we3 = 0.0
	)

	if m.WValid {
		ww := math.Sqrt(s.W1S * s.W1S + s.W2S * s.W2S + s.W3S * s.W3S)
		if ww > 10 {
			we1 = s.W1S / ww
			we2 = s.W2S / ww
			we3 = s.W3S / ww
		}
	}

	// Compute sensor forward direction in earth frame
	xe1 := 1 - 2 * (q3 * q3 + q2 * q2)
	xe2 := 2 * (q0 * q3 + q1 * q2)
	xe3 := 2 * (q1 * q3 - q0 * q2)

	// Now the angle to rotate around ae to minimize diff:
	// Construct a vector perpendicular to ae and xe:
	u1 := ae2 * xe3 - ae3 * xe2
	u2 := ae3 * xe1 - ae1 * xe3
	u3 := ae1 * xe2 - ae2 * xe1
	uu := math.Sqrt(u1 * u1 + u2 * u2 + u3 * u3)
	u1 /= uu
	u2 /= uu
	u3 /= uu
	// Construct a vector perpendicular to ae and we:
	v1 := ae2 * we3 - ae3 * we2
	v2 := ae3 * we1 - ae1 * we3
	v3 := ae1 * we2 - ae2 * we1
	vv := math.Sqrt(v1 * v1 + v2 * v2 + v3 * v3)
	v1 /= vv
	v2 /= vv
	v3 /= vv
	alpha := math.Acos(u1 * v1 + u2 * v2 + u3 * v3)
	log.Printf("AHRS: alpha is %2.1f\n", alpha*180/Pi)
	//TODO westphae: figure out the correct sign for alpha

	// Update the qea quaternion by rotating around ae with this angle
	// Construct ae rotation quaternion:
	p0 = math.Cos(alpha / 2)
	sa := math.Sin(alpha / 2)
	ae := math.Sqrt(ae1*ae1 + ae2*ae2 + ae3*ae3)
	p1 = sa * ae1 / ae
	p2 = sa * ae2 / ae
	p3 = sa * ae3 / ae

	// Rotate around ae to get our final orientation quaternion:
	s.E0 = +(p0 * q0 - p1 * q1 - p2 * p2 - p3 * q3)
	s.E1 = -(p0 * q1 + p1 * q0 + p2 * q3 - p3 * q2)
	s.E2 = -(p0 * q2 - p1 * q3 + p2 * q0 + p3 * q1)
	s.E3 = -(p0 * q3 + p1 * q2 - p2 * q1 + p3 * q0)

	// Check that sign of alpha was correct
	if (1-2*(s.E3*s.E3+s.E2*s.E2))*we1 + (2*(s.E0*s.E3+s.E1*s.E2))*we2 + (2*(s.E1*s.E3-s.E0*s.E2))*we3 < 0 {
		s.E0 -= 2*p0*q0
		s.E1 += 2*p0*q1
		s.E2 += 2*p0*q2
		s.E3 += 2*p0*q3
	}

	// Save the current GPS speeds for next loop
	s.W10 = m.W1
	s.W20 = m.W2
	s.W30 = m.W3
	s.T = m.T

	if m.MValid { //TODO westphae: could do more here to get a better Fn since we know N points north
		s.N1 = m.M1*s.e11 + m.M2*s.e12 + m.M3*s.e13
		s.N2 = m.M1*s.e21 + m.M2*s.e22 + m.M3*s.e23
		s.N3 = m.M1*s.e31 + m.M2*s.e32 + m.M3*s.e33
	}
}

func (s *HeuristicState) Valid() (ok bool) {
	return true
}

func (s *HeuristicState) CalcRollPitchHeading() (roll float64, pitch float64, heading float64) {
	roll, pitch, heading = FromQuaternion(s.E0, s.E1, s.E2, s.E3)
	return
}

func (s *HeuristicState) CalcRollPitchHeadingUncertainty() (droll float64, dpitch float64, dheading float64) {

	return
}

func (s *HeuristicState) PredictMeasurement() *Measurement {
	return NewMeasurement()
}

