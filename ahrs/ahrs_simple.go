package ahrs

import (
	_ "log"
	"math"
	"github.com/skelterjohn/go.matrix"
)

const (
	minDT        float64 = 1e-6 // Below this time interval, don't recalculate
	maxDT        float64 = 10   // Above this time interval, re-initialize--too stale
	minGS        float64 = 10   // Below this GS, don't use any GPS data
	rollBand     float64 = 5 // Degrees by which roll can differ from pitchGPS
	pitchBand    float64 = 5 // Degrees by which pitch can differ from pitchGPS
	headingBand  float64 = 5 // Degrees by which heading can differ from pitchGPS
	gpsTimeConst float64 = 15 // Seconds time constant for attitude to decay towards GPS value without gyro input
)

type SimpleState struct {
	State
	rollGPS, pitchGPS, headingGPS float64 // GPS-derived attitude, Deg
	roll, pitch, heading          float64 // Fused attitude, Deg
	w1, w2, w3, gs                float64 // Groundspeed & ROC tracking, Kts
	tr                            float64 // turn rate, Rad/s
}

func InitializeSimple(m *Measurement) (s *SimpleState) {
	s = new(SimpleState)
	s.M = matrix.Zeros(32, 32)
	s.N = matrix.Zeros(32, 32)
	s.init(m)
	return
}

func (s *SimpleState) init(m *Measurement) {
	s.T = m.T
	if m.WValid {
		s.gs = math.Hypot(m.W1, m.W2)
		s.w1 = m.W1
		s.w2 = m.W2
		s.w3 = m.W3
	} else {
		s.gs = 0
		s.w1 = 0
		s.w2 = 0
		s.w3 = 0
	}

	s.tr = 0
	s.rollGPS = 0
	if s.gs > minGS {
		s.headingGPS = math.Atan2(m.W1, m.W2)
		s.pitchGPS = math.Atan2(m.W3, s.gs)
	} else {
		s.headingGPS = Pi/2
		s.pitchGPS = 0
	}

	s.roll = s.rollGPS
	s.pitch = s.pitchGPS
	s.heading = s.headingGPS

	s.E0, s.E1, s.E2, s.E3 = ToQuaternion(s.roll, s.pitch, s.heading)
}

func (s *SimpleState) Compute(m *Measurement) {
	s.Predict(m.T)
	s.Update(m)
}

func (s *SimpleState) Predict(t float64) {
	return
}

func (s *SimpleState) Update(m *Measurement) {
	dt := m.T - s.T
	if dt < minDT {
		return
	}
	if dt > maxDT {
		s.init(m)
		return
	}

	if m.WValid {
		s.gs = math.Hypot(m.W1, m.W2)
	}

	if m.WValid && s.gs > minGS {
		s.tr = 0.9*s.tr + 0.1*(m.W2*(m.W1-s.w1)-m.W1*(m.W2-s.w2))/(s.gs*s.gs)/dt
		s.rollGPS = math.Atan(s.gs*s.tr/G)
		s.pitchGPS = math.Atan2(m.W3, s.gs)
		s.headingGPS = math.Atan2(m.W1, m.W2)
		s.w1 = m.W1
		s.w2 = m.W2
		s.w3 = m.W3
	} else {
		s.tr = 0
		s.rollGPS = 0
		s.pitchGPS = 0
		s.headingGPS = Pi/2
		s.roll = 0
		s.pitch = 0
		s.heading = Pi/2
		s.w1 = 0
		s.w2 = 0
		s.w3 = 0
	}

	q0, q1, q2, q3 := s.E0, s.E1, s.E2, s.E3
	dq0, dq1, dq2, dq3 := QuaternionRotate(q0, q1, q2, q3, m.B1*Deg*dt, m.B2*Deg*dt, m.B3*Deg*dt)
	dq0 -= q0
	dq1 -= q1
	dq2 -= q2
	dq3 -= q3

	rx := 2 * (q0*q1 + q2*q3)
	ry := q0*q0 - q1*q1 - q2*q2 + q3*q3
	drx := 2 * (q1*dq0 + q0*dq1 + q3*dq2 + q2*dq3)
	dry := 2 * (q0*dq0 - q1*dq1 - q2*dq2 + q3*dq3)
	dr := (ry*drx - rx*dry) / (rx*rx + ry*ry)

	px := -2 * (q0*q2 - q3*q1)
	py := q0*q0 + q1*q1 + q2*q2 + q3*q3
	dpx := -2 * (q2*dq0 - q3*dq1 + q0*dq2 - q1*dq3)
	dpy := 2 * (q0*dq0 + q1*dq1 + q2*dq2 + q3*dq3)
	dp := (py*dpx - px*dpy) / (py * math.Sqrt(py*py-px*px))

	hx := -2 * (q0*q3 - q1*q2)
	hy := q0*q0 + q1*q1 - q2*q2 - q3*q3
	dhx := -2 * (q3*dq0 - q2*dq1 - q1*dq2 + q0*dq3)
	dhy := 2 * (q0*dq0 + q1*dq1 - q2*dq2 - q3*dq3)
	dh := (hy*dhx - hx*dhy) / (hx*hx + hy*hy)

	// This won't work around the poles -- no hammerheads!
	kp := 0.5 * (s.pitch - s.pitchGPS) / pitchBand + 0.5
	// The idea of the simple AHRS is to bias the sensors to bring the estimated attitude
	// in line with the GPS-derived attitude
	if (s.pitch-s.pitchGPS)*dp > 0 {
		dp *= kp
	}

	kr := 0.5 * (s.roll - s.rollGPS) / rollBand + 0.5
	if (s.roll-s.rollGPS)*dr > 0 {
		dr *= kr
	}

	kh := 0.5 * (s.heading - s.headingGPS) / headingBand + 0.5
	ddh := s.heading - s.headingGPS
	if ddh > Pi {
		ddh -= 2*Pi
	} else if ddh < -Pi {
		ddh += 2*Pi
	}
	if ddh*dh > 0 {
		dh *= kh
	}

	s.pitch += dp + (s.pitchGPS-s.pitch)*dt/gpsTimeConst
	s.roll += dr + (s.rollGPS-s.roll)*dt/gpsTimeConst
	s.heading += dh - ddh*dt/gpsTimeConst

	s.roll, s.pitch, s.heading = Regularize(s.roll, s.pitch, s.heading)
	s.rollGPS, s.pitchGPS, s.headingGPS = Regularize(s.rollGPS, s.pitchGPS, s.headingGPS)

	s.E0, s.E1, s.E2, s.E3 = ToQuaternion(s.roll, s.pitch, s.heading)
	s.T = m.T
}

func (s *SimpleState) Valid() (ok bool) {
	return true
}

func (s *SimpleState) CalcRollPitchHeading() (roll float64, pitch float64, heading float64) {
	roll, pitch, heading = FromQuaternion(s.E0, s.E1, s.E2, s.E3)
	return
}

func (s *SimpleState) CalcGPSRollPitchHeading() (roll float64, pitch float64, heading float64) {
	roll, pitch, heading = s.rollGPS, s.pitchGPS, s.headingGPS
	return
}

func (s *SimpleState) CalcRollPitchHeadingUncertainty() (droll float64, dpitch float64, dheading float64) {
	return
}

// GetStateMap returns the state information for analysis
func (s *SimpleState) GetStateMap() (dat *map[string]float64) {
	phi, theta, psi := FromQuaternion(s.E0, s.E1, s.E2, s.E3)
	dat = &map[string]float64{
		"T":  s.T,
		"E0": s.E0,
		"E1": s.E1,
		"E2": s.E2,
		"E3": s.E3,
		"Phi": phi / Deg,
		"Theta": theta / Deg,
		"Psi": psi / Deg,
		"rollGPS": s.rollGPS,
		"pitchGPS": s.pitchGPS,
		"headingGPS": s.headingGPS,
		"roll": s.roll,
		"pitch": s.pitch,
		"heading": s.heading,
		"W1": s.w1,
		"W2": s.w2,
		"W3": s.w3,
		"GS": s.gs,
		"TR": s.tr,
	}
	return
}

// PredictMeasurement doesn't do anything for the Simple method
func (s *SimpleState) PredictMeasurement() *Measurement {
	return NewMeasurement()
}
