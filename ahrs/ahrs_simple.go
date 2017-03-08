package ahrs

import (
	"log"
	"math"

	"github.com/skelterjohn/go.matrix"
)

const (
	minDT         float64 = 1e-6 // Below this time interval, don't recalculate
	maxDT         float64 = 10   // Above this time interval, re-initialize--too stale
	minGS         float64 = 10   // Below this GS, don't use any GPS data
	rollBand      float64 = 10   // Degrees by which roll can differ from pitchGPS
	pitchBand     float64 = 10   // Degrees by which pitch can differ from pitchGPS
	headingBand   float64 = 10   // Degrees by which heading can differ from pitchGPS
	expPower      float64 = 5
	gpsTimeConst  float64 = 5    // Seconds time constant for attitude to decay towards GPS value without gyro input
	bCalTimeConst float64 = 600  // Time constant for calibrating gyro, s
	trSmall       float64 = 0.25 * Deg  // Turn Rate that we will consider to be zero for gyro calibration
	warmupTime    float64 = 60   // Time after beginning of flight to accumulate gyro calibration more quickly
	gpsSmoothConst float64 = 0.7 // Multiplier for exponential smoothing of GPS-derived values
)

type SimpleState struct {
	State
	TW                            float64                // Time of last GPS reading
	rollGPS, pitchGPS, headingGPS float64                // GPS-derived attitude, Deg
	roll, pitch, heading          float64                // Fused attitude, Deg
	w1, w2, w3, gs                float64                // Groundspeed & ROC tracking, Kts
	tr                            float64                // turn rate, Rad/s
	headingMag                    float64                // Magnetic heading, Rad (smoothed)
	slipSkid                      float64                // Slip/Skid Angle, Rad (smoothed)
	gLoad                         float64                // G Load, G vertical (smoothed)
	turnRate                      float64                // turn rate, Rad/s (smoothed)
	calTime                       float64                // Time since beginning of flight, s
	needsInitialization           bool                   // Rather than computing, initialize
	logMap                        map[string]interface{} // Map only for analysis/debugging
}

func InitializeSimple() (s *SimpleState) {
	s = new(SimpleState)
	s.logMap = make(map[string]interface{})
	updateLogMap(s, NewMeasurement(), s.logMap)
	s.needsInitialization = true
	s.M = matrix.Zeros(32, 32)
	s.N = matrix.Zeros(32, 32)
	return
}

func (s *SimpleState) init(m *Measurement) {
	s.needsInitialization = false
	s.T = m.T
	s.TW = m.TW
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

	s.calTime = 0
	s.D1, s.D2, s.D3 = 0, 0, 0

	updateLogMap(s, m, s.logMap)
}

func (s *SimpleState) Compute(m *Measurement) {
	if s.needsInitialization {
		s.init(m)
	} else {
		s.Predict(m.T)
		s.Update(m)
	}
}

func (s *SimpleState) Predict(t float64) {
	return
}

func (s *SimpleState) Update(m *Measurement) {
	dt := m.T - s.T
	dtw := m.TW - s.TW

	if dt > maxDT || dtw > maxDT {
		log.Printf("AHRS Info: Reinitializing at %f\n", m.T)
		s.init(m)
		return
	}

	if m.WValid && dtw > minDT {
		s.gs = math.Hypot(m.W1, m.W2)
	}

	var rollGPS, pitchGPS, headingGPS float64
	if s.gs > minGS {
		if dtw > minDT {
			s.tr = (m.W2*(m.W1-s.w1)-m.W1*(m.W2-s.w2))/(s.gs*s.gs)/dtw
			rollGPS = math.Atan(s.gs * s.tr / G)
			pitchGPS = math.Atan2(m.W3, s.gs)
			headingGPS = math.Atan2(m.W1, m.W2)
		} else {
			log.Printf("GPS time interval too short at %f\n", m.T)
			rollGPS = s.rollGPS
			pitchGPS = s.pitchGPS
			headingGPS = s.headingGPS
		}
	} else {
		s.tr = 0
		rollGPS = math.Atan2(-m.A2, -m.A3) // Needs to be different with pitch != 0
		pitchGPS = math.Asin(-m.A1/math.Sqrt(m.A1*m.A1 + m.A2*m.A2 + m.A3*m.A3))
		headingGPS = s.heading
		s.calTime = 0 // TODO westphae: need something more sophisticated with static mode?
	}
	s.rollGPS = gpsSmoothConst * s.rollGPS + (1-gpsSmoothConst) * rollGPS
	s.pitchGPS = gpsSmoothConst * s.pitchGPS + (1-gpsSmoothConst) * pitchGPS
	s.headingGPS = gpsSmoothConst * s.headingGPS + (1-gpsSmoothConst) * headingGPS

	q0, q1, q2, q3 := s.E0, s.E1, s.E2, s.E3
	dq0, dq1, dq2, dq3 := QuaternionRotate(q0, q1, q2, q3,
		(m.B1-s.D1)*Deg*dt, (m.B2-s.D2)*Deg*dt, (m.B3-s.D3)*Deg*dt)
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
	kp := math.Exp(-math.Abs((s.pitch - s.pitchGPS) / pitchBand)*expPower) // linear
	// The idea of the simple AHRS is to bias the sensors to bring the estimated attitude
	// in line with the GPS-derived attitude
	if (s.pitch-s.pitchGPS)*dp > 0 {
		dp *= math.Max(0, kp)
	}

	kr := math.Exp(-math.Abs((s.roll - s.rollGPS) / rollBand)*expPower) // linear
	if (s.roll-s.rollGPS)*dr > 0 {
		dr *= math.Max(0, kr)
	}

	kh := math.Exp(-math.Abs((s.heading - s.headingGPS) / headingBand)*expPower) // linear
	ddh := s.heading - s.headingGPS
	if ddh > Pi {
		ddh -= 2*Pi
	} else if ddh < -Pi {
		ddh += 2*Pi
	}
	if ddh*dh > 0 {
		dh *= math.Max(0, kh)
	}

	s.pitch += dp + (s.pitchGPS-s.pitch)*dt/gpsTimeConst
	s.roll += dr + (s.rollGPS-s.roll)*dt/gpsTimeConst
	s.heading += dh - ddh*dt/gpsTimeConst

	s.roll, s.pitch, s.heading = Regularize(s.roll, s.pitch, s.heading)
	s.rollGPS, s.pitchGPS, s.headingGPS = Regularize(s.rollGPS, s.pitchGPS, s.headingGPS)

	s.E0, s.E1, s.E2, s.E3 = ToQuaternion(s.roll, s.pitch, s.heading)

	// Update Magnetic Heading
	hM := math.Atan2(m.M1, -m.M2)
	if hM-s.headingMag < -Pi {
		hM += 2*Pi
	}
	s.headingMag = gpsSmoothConst * s.headingMag + (1-gpsSmoothConst) * hM
	for s.headingMag < 0 {
		s.headingMag += 2*Pi
	}
	for s.headingMag >= 2*Pi {
		s.headingMag -= 2*Pi
	}

	// Update Slip/Skid
	s.slipSkid = gpsSmoothConst * s.slipSkid + (1-gpsSmoothConst) * math.Atan2(m.A2, -m.A3)

	// Update Rate of Turn
	s.turnRate = gpsSmoothConst * s.turnRate + (1-gpsSmoothConst) * s.tr

	// Update GLoad
	s.gLoad = gpsSmoothConst * s.gLoad + (1-gpsSmoothConst) * -m.A3

	// Recalibrate
	if math.Abs(s.tr) < trSmall {
		w := dt/bCalTimeConst
		if s.calTime < warmupTime {
			w *= 10 // Accumulate calibration values more quickly at beginning
			s.calTime += dt
		}
		s.D1 = (1-w) * s.D1 + w * m.B1
		s.D2 = (1-w) * s.D2 + w * m.B2
		s.D3 = (1-w) * s.D3 + w * m.B3
	}

	updateLogMap(s, m, s.logMap)

	s.T = m.T
	s.TW = m.TW
	s.w1 = m.W1
	s.w2 = m.W2
	s.w3 = m.W3
}

func (s *SimpleState) Valid() (ok bool) {
	return true
}

func (s *SimpleState) RollPitchHeading() (roll float64, pitch float64, heading float64) {
	roll, pitch, heading = FromQuaternion(s.E0, s.E1, s.E2, s.E3)
	return
}

// MagHeading returns the magnetic heading in degrees.
func (s *SimpleState) MagHeading() (hdg float64) {
	return s.headingMag / Deg
}

// SlipSkid returns the slip/skid angle in degrees.
func (s *SimpleState) SlipSkid() (slipSkid float64) {
	return s.slipSkid / Deg
}

// RateOfTurn returns the turn rate in degrees per second.
func (s *SimpleState) RateOfTurn() (turnRate float64) {
	return s.turnRate / Deg
}

// GLoad returns the current G load, in G's.
func (s *SimpleState) GLoad() (gLoad float64) {
	return s.gLoad
}

func (s *SimpleState) Reset() {
	s.needsInitialization = true
}

// PredictMeasurement doesn't do anything for the Simple method
func (s *SimpleState) PredictMeasurement() *Measurement {
	return NewMeasurement()
}

// GetState returns the Kalman state of the system
func (s *SimpleState) GetState() *State {
	return &s.State
}

// GetLogMap returns a map providing current state and measurement values for analysis
func (s *SimpleState) GetLogMap() (p map[string]interface{}) {
	return s.logMap
}

func updateLogMap(s *SimpleState, m *Measurement, p map[string]interface{}) {
	var simpleLogMap = map[string]func(s *SimpleState, m *Measurement) float64{
		"Ta":          func(s *SimpleState, m *Measurement) float64 { return s.T },
		"TWa":         func(s *SimpleState, m *Measurement) float64 { return s.TW },
		"calTime":     func(s *SimpleState, m *Measurement) float64 { return s.calTime },
		"Roll":        func(s *SimpleState, m *Measurement) float64 { return s.roll / Deg },
		"Pitch":       func(s *SimpleState, m *Measurement) float64 { return s.pitch / Deg },
		"Heading":     func(s *SimpleState, m *Measurement) float64 { return s.heading / Deg },
		"RollGPS":     func(s *SimpleState, m *Measurement) float64 { return s.rollGPS / Deg },
		"PitchGPS":    func(s *SimpleState, m *Measurement) float64 { return s.pitchGPS / Deg },
		"HeadingGPS":  func(s *SimpleState, m *Measurement) float64 { return s.headingGPS / Deg },
		"tr":          func(s *SimpleState, m *Measurement) float64 { return s.tr },
		"GroundSpeed": func(s *SimpleState, m *Measurement) float64 { return s.gs },
		"W1a":         func(s *SimpleState, m *Measurement) float64 { return s.w1 },
		"W2a":         func(s *SimpleState, m *Measurement) float64 { return s.w2 },
		"W3a":         func(s *SimpleState, m *Measurement) float64 { return s.w3 },
		"dPitch":      func(s *SimpleState, m *Measurement) float64 { return pitchBand },
		"dRoll":       func(s *SimpleState, m *Measurement) float64 { return rollBand },
		"dHeading":    func(s *SimpleState, m *Measurement) float64 { return headingBand },
		"WValid": func(s *SimpleState, m *Measurement) float64 { if m.WValid { return 1 }; return 0 },
		"T":  func(s *SimpleState, m *Measurement) float64 { return m.T },
		"TW": func(s *SimpleState, m *Measurement) float64 { return m.TW },
		"W1": func(s *SimpleState, m *Measurement) float64 { return m.W1 },
		"W2": func(s *SimpleState, m *Measurement) float64 { return m.W2 },
		"W3": func(s *SimpleState, m *Measurement) float64 { return m.W3 },
		"E0": func(s *SimpleState, m *Measurement) float64 { return s.E0 },
		"E1": func(s *SimpleState, m *Measurement) float64 { return s.E1 },
		"E2": func(s *SimpleState, m *Measurement) float64 { return s.E2 },
		"E3": func(s *SimpleState, m *Measurement) float64 { return s.E3 },
		"A1": func(s *SimpleState, m *Measurement) float64 { return m.A1 },
		"A2": func(s *SimpleState, m *Measurement) float64 { return m.A2 },
		"A3": func(s *SimpleState, m *Measurement) float64 { return m.A3 },
		"B1": func(s *SimpleState, m *Measurement) float64 { return m.B1 },
		"B2": func(s *SimpleState, m *Measurement) float64 { return m.B2 },
		"B3": func(s *SimpleState, m *Measurement) float64 { return m.B3 },
		"M1": func(s *SimpleState, m *Measurement) float64 { return m.M1 },
		"M2": func(s *SimpleState, m *Measurement) float64 { return m.M2 },
		"M3": func(s *SimpleState, m *Measurement) float64 { return m.M3 },
		"D1": func(s *SimpleState, m *Measurement) float64 { return s.D1 },
		"D2": func(s *SimpleState, m *Measurement) float64 { return s.D2 },
		"D3": func(s *SimpleState, m *Measurement) float64 { return s.D3 },
		"headingMag": func(s *SimpleState, m *Measurement) float64 { return s.headingMag },
		"slipSkid": func(s *SimpleState, m *Measurement) float64 { return s.slipSkid },
		"gLoad": func(s *SimpleState, m *Measurement) float64 { return s.gLoad },
		"turnRate": func(s *SimpleState, m *Measurement) float64 { return s.turnRate },
	}

	for k := range simpleLogMap {
		p[k] = simpleLogMap[k](s, m)
	}
}

var SimpleJSONConfig = `
{
  "State": [
    {"pred": "GPSRoll", "updt": "Roll", "std": "dRoll", "baseline": 0},
    {"pred": "GPSPitch", "updt": "Pitch", "std": "dPitch", "baseline": 0},
    {"pred": "GPSHeading", "updt": "Heading", "std": "dHeading"},
    {"updt": "TurnRate", "baseline": 0},
    {"updt": "GroundSpeed", "baseline": 0},
    {"updt": "T"},
    {"updt": "D1", "baseline": 0},
    {"updt": "D2", "baseline": 0},
    {"updt": "D3", "baseline": 0}
  ],
  "Measurement": [
    {"meas": "W1", "pred": "W1a", "baseline": 0},
    {"meas": "W2", "pred": "W2a", "baseline": 0},
    {"meas": "W3", "pred": "W3a", "baseline": 0},
    {"meas": "A1", "baseline": 0},
    {"meas": "A2", "baseline": 0},
    {"meas": "A3", "baseline": 1},
    {"meas": "B1", "baseline": 0},
    {"meas": "B2", "baseline": 0},
    {"meas": "B3", "baseline": 0},
    {"meas": "M1", "baseline": 0},
    {"meas": "M2", "baseline": 0},
    {"meas": "M3", "baseline": 0}
  ]
}
`
