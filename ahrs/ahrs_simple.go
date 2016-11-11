package ahrs

import (
	"math"
)

const (
	K = 0.9	// Reversion constant
)

type SimpleState struct {
	State
	rollGPS, pitchGPS, headingGPS float64 // Deg
	roll, pitch, heading          float64 // Deg
	w1, w2, gs                    float64 // Kts
	tr                            float64 // Rad/s
}

func (s *SimpleState) init(m *Measurement) {
	s.T = m.T
	if m.WValid {
		s.gs = math.Hypot(m.W1, m.W2)
		s.w1 = m.W1
		s.w2 = m.W2
	} else {
		s.gs = 0
		s.w1 = 0
		s.w2 = 0
	}

	s.tr = 0
	s.rollGPS = 0
	s.pitchGPS = 0
	if s.gs > 10 {
		s.headingGPS = math.Atan2(m.W1, m.W2) / Deg
	} else {
		s.headingGPS = 90
	}

	s.roll = s.rollGPS
	s.pitch = s.pitchGPS
	s.heading = s.headingGPS
}

func InitializeSimple(m *Measurement) (s *SimpleState) {
	s = new(SimpleState)
	s.init(m)
	return
}

func (s *SimpleState) Compute(m *Measurement) {
	dt := m.T - s.T
	if dt < 1e-6 {
		return
	}
	if dt > 10 {
		s.init(m)
		return
	}

	if m.WValid {
		s.gs = math.Hypot(m.W1, m.W2)
	}

	if m.WValid && s.gs > 10 {
		s.tr = 0.9*s.tr + 0.1*(m.W2 * (m.W1 - s.w1) - m.W1 * (m.W2 - s.w2)) / (s.gs * s.gs)/ dt
		s.rollGPS = math.Atan(s.gs*s.tr/G) / Deg
		s.pitchGPS = 0 //TODO westphae: can we do better here?
		s.headingGPS = math.Atan2(m.W1, m.W2) / Deg
		s.w1 = m.W1
		s.w2 = m.W2
	} else {
		s.tr = 0
		s.rollGPS = s.roll
		s.pitchGPS = s.pitch
		s.headingGPS = s.heading
		s.w1 = 0
		s.w2 = 0
	}

	dr := m.B1*dt
	dp := (-math.Cos(s.rollGPS*Deg)*m.B2 + math.Sin(s.rollGPS*Deg)*m.B3)*dt
	dh := (-math.Sin(s.rollGPS*Deg)*m.B2 - math.Cos(s.rollGPS*Deg)*m.B3)*dt

	if (s.pitch - s.pitchGPS)*dp > 0 {
		dp *= K
	}
	if (s.roll - s.rollGPS)*dr > 0 {
		dr *= K
	}
	ddh := s.heading - s.headingGPS
	if ddh > 180 {
		ddh -= 360
	} else if ddh < -180 {
		ddh += 360
	}
	if ddh*dh > 0 {
		dh *= K
	}

	s.pitch += dp
	s.roll += dr
	s.heading += dh

	// Regularize
	for s.pitch > 180 {
		s.pitch -= 360
	}
	for s.pitch <= -180 {
		s.pitch += 360
	}
	if s.pitch > 90 {
		s.pitch = 180 - s.pitch
		s.roll -= 180
		s.heading += 180
	}
	if s.pitch < -90 {
		s.pitch = -180 - s.pitch
		s.roll -= 180
		s.heading += 180
	}

	for s.roll > 180 {
		s.roll -= 360
	}
	for s.roll < -180 {
		s.roll += 360
	}

	for s.heading >= 360 {
		s.heading -= 360
	}
	for s.heading < 0 {
		s.heading += 360
	}

	s.E0, s.E1, s.E2, s.E3 = ToQuaternion(s.roll*Deg, s.pitch*Deg, s.heading*Deg)
	s.T = m.T
}

func (s *SimpleState) Valid() (ok bool) {
	return true
}

func (s *SimpleState) CalcRollPitchHeading() (roll float64, pitch float64, heading float64) {
	roll, pitch, heading = s.roll, s.pitch, s.heading
	return
}

func (s *SimpleState) CalcRollPitchHeadingUncertainty() (droll float64, dpitch float64, dheading float64) {
	return
}
