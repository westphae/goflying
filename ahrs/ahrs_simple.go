package ahrs

import (
	"log"
	"math"

	"github.com/skelterjohn/go.matrix"
)

const (
	minDT = 1e-6                     // Below this time interval, don't recalculate
	maxDT = 10.0                     // Above this time interval, re-initialize--too stale
	minGS = 5.0                      // Below this GS, don't use any GPS data
	smoothConstGS = 0.02             // Five-second smoothing for groundspeed, to decide static mode
	uiSmoothConstDefault = 0.65      // Sensible default for smoothing AHRS values
	slipSkidSmoothConstDefault = 0.1 // Sensible default for smoothing AHRS values
	gpsWeightDefault = 0.025         // Sensible default for weight of GPS solution
)

var (
	uiSmoothConst = uiSmoothConstDefault              // Decay constant for smoothing values reported to the user
	slipSkidSmoothConst = slipSkidSmoothConstDefault  // Decay constant for smoothing values reported to the user
	gpsWeight = gpsWeightDefault                      // Weight given to GPS quaternion over gyro quaternion
)

type SimpleState struct {
	State
	TW                            float64                // Time of last GPS reading
	EGPS0, EGPS1, EGPS2, EGPS3    float64                // GPS-derived orientation quaternion
	roll, pitch, heading          float64                // Fused attitude, Rad
	rollGPS, pitchGPS, headingGPS float64                // GPS/accel-based attitude, Rad
	w1, w2, w3, gs                float64                // Groundspeed & ROC, Kts
	smoothW1, smoothW2, smoothGS  float64                // Smoothed groundspeed used to determine if stationary
	headingMag                    float64                // Magnetic heading, Rad (smoothed)
	slipSkid                      float64                // Slip/Skid Angle, Rad (smoothed)
	gLoad                         float64                // G Load, G vertical (smoothed)
	turnRate                      float64                // turn rate, Rad/s (smoothed)
	needsInitialization           bool                   // Rather than computing, initialize
	staticMode                    bool                   // For low groundspeed or invalid GPS
	headingValid                  bool                   // Whether to slew quickly to correct heading
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
	s.headingValid = false
	s.T = m.T
	s.TW = m.TW
	if m.WValid {
		s.gs = math.Hypot(m.W1, m.W2)
		s.smoothW1 = s.smoothW1 + smoothConstGS*(m.W1 - s.smoothW1)
		s.smoothW2 = s.smoothW2 + smoothConstGS*(m.W2 - s.smoothW2)
		s.smoothGS = math.Hypot(s.smoothW1, s.smoothW2)
		s.w1 = m.W1
		s.w2 = m.W2
		s.w3 = m.W3
	} else {
		s.gs = 0
		s.smoothW1 = 0
		s.smoothW2 = 0
		s.smoothGS = 0
		s.w1 = 0
		s.w2 = 0
		s.w3 = 0
	}

	s.roll = 0
	s.pitch = 0
	if s.smoothGS > minGS {
		s.heading = math.Atan2(m.W1, m.W2)
		for s.heading < 0 {
			s.heading += 2*Pi
		}
		for s.heading >= 2*Pi {
			s.heading -= 2 * Pi
		}
	} else {
		s.heading = 0
	}

	s.EGPS0, s.EGPS1, s.EGPS2, s.EGPS3 = ToQuaternion(s.roll, s.pitch, s.heading)

	s.E0, s.E1, s.E2, s.E3 = s.EGPS0, s.EGPS1, s.EGPS2, s.EGPS3

	s.D1, s.D2, s.D3 = 0, 0, 0

	// Update Magnetic Heading
	s.headingMag = math.Atan2(m.M1, -m.M2)
	for s.headingMag < 0 {
		s.headingMag += 2 * Pi
	}
	for s.headingMag >= 2*Pi {
		s.headingMag -= 2 * Pi
	}

	// Update Slip/Skid
	s.slipSkid = math.Atan2(m.A2, -m.A3)

	// Update Rate of Turn
	s.turnRate = 0

	// Update GLoad
	s.gLoad += uiSmoothConst * (-m.A3 - s.gLoad)

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

// Update performs the AHRSSimple AHRS computations.
// The idea behind the Simple AHRS algorithm is to use the GPS to compute what the accelerometer should show
// and then to create a rotation matrix to map the measured accelerometer vector onto this vector and the
// speed vector (GPS Track) onto the sensor x-axis.  Then the gyro is used to further improve this estimate.
// This is really a poor-man's sensor fusion algorithm.  The proper way to do this is with a Kalman Filter,
// but this approach is simpler and easier to debug, and should be good enough for most flight conditions.
//
// It is a step on the way to the full Kalman Filter implementation, a bit more obvious what's going on so
// the math and stratux integration can be more easily developed and debugged.
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
		s.smoothW1 = s.smoothW1 + smoothConstGS*(m.W1 - s.smoothW1)
		s.smoothW2 = s.smoothW2 + smoothConstGS*(m.W2 - s.smoothW2)
		s.smoothGS = math.Hypot(s.smoothW1, s.smoothW2)
	}

	ae := [3]float64{0, 0, -1} // Acceleration due to gravity in earth frame
	ve := [3]float64{0, 1, 0}  // Groundspeed in earth frame (default for desktop mode)
	s.staticMode = !(m.WValid && (s.smoothGS > minGS))
	if !s.staticMode {
		if !s.headingValid  {
			s.init(m)
			s.headingValid = true
			return
		}
		if dtw < minDT {
			log.Printf("No GPS update at %f\n", m.T)
			return
		}
		ve = [3]float64{m.W1, m.W2, m.W3} // Instantaneous roundspeed in earth frame
		// Instantaneous acceleration in earth frame based on change in GPS groundspeed
		ae[0] -= (m.W1 - s.w1) / dtw / G
		ae[1] -= (m.W2 - s.w2) / dtw / G
		ae[2] -= (m.W3 - s.w3) / dtw / G
	}

	ha, err := MakeUnitVector([3]float64{m.A1, m.A2, m.A3})
	if err != nil {
		log.Println("AHRS Error: IMU-measured acceleration was zero")
		return
	}

	he, _ := MakeUnitVector(ae)
	se, _ := MakeUnitVector(ve)

	// Left-multiplying a vector in the aircraft frame by rotmat will put it into the earth frame.
	// rotmat maps the current IMU acceleration to the GPS-acceleration and the x-axis to the GPS-velocity.
	rotmat, err := MakeHardSoftRotationMatrix(*ha, [3]float64{1, 0, 0}, *he, *se)
	if err != nil {
		log.Printf("AHRS Error: %s\n", err)
		return
	}

	// This orientation quaternion EGPS rotates from aircraft frame to earth frame at the current time,
	// as estimated using GPS and accelerometer.
	e0, e1, e2, e3 := RotationMatrixToQuaternion(*rotmat)
	e0, e1, e2, e3 = QuaternionSign(e0, e1, e2, e3, s.EGPS0, s.EGPS1, s.EGPS2, s.EGPS3)
	s.EGPS0, s.EGPS1, s.EGPS2, s.EGPS3 = QuaternionNormalize(
		s.EGPS0+uiSmoothConst*(e0-s.EGPS0),
		s.EGPS1+uiSmoothConst*(e1-s.EGPS1),
		s.EGPS2+uiSmoothConst*(e2-s.EGPS2),
		s.EGPS3+uiSmoothConst*(e3-s.EGPS3),
	)

	// By rotating the orientation quaternion at the last time step, s.E, by the measured gyro rates,
	// we get another estimate of the current orientation quaternion using gyro.
	e0, e1, e2, e3 = QuaternionRotate(s.E0, s.E1, s.E2, s.E3,
		(m.B1-s.D1)*dt*Deg, (m.B2-s.D2)*dt*Deg, (m.B3-s.D3)*dt*Deg)

	// Now fuse the GPS/Accelerometer and Gyro estimates, smooth the result and normalize.
	s.EGPS0, s.EGPS1, s.EGPS2, s.EGPS3 = QuaternionSign(s.EGPS0, s.EGPS1, s.EGPS2, s.EGPS3, s.E0, s.E1, s.E2, s.E3)
	s.E0, s.E1, s.E2, s.E3 = QuaternionNormalize(
		s.E0+uiSmoothConst*(gpsWeight*s.EGPS0+(1-gpsWeight)*e0-s.E0),
		s.E1+uiSmoothConst*(gpsWeight*s.EGPS1+(1-gpsWeight)*e1-s.E1),
		s.E2+uiSmoothConst*(gpsWeight*s.EGPS2+(1-gpsWeight)*e2-s.E2),
		s.E3+uiSmoothConst*(gpsWeight*s.EGPS3+(1-gpsWeight)*e3-s.E3),
	)

	s.roll, s.pitch, s.heading = FromQuaternion(s.E0, s.E1, s.E2, s.E3)
	s.rollGPS, s.pitchGPS, s.headingGPS = FromQuaternion(s.EGPS0, s.EGPS1, s.EGPS2, s.EGPS3)

	// Update Magnetic Heading
	dhM := AngleDiff(math.Atan2(m.M1, -m.M2), s.headingMag)
	s.headingMag += uiSmoothConst * dhM
	for s.headingMag < 0 {
		s.headingMag += 2 * Pi
	}
	for s.headingMag >= 2*Pi {
		s.headingMag -= 2 * Pi
	}

	// Update Slip/Skid
	s.slipSkid += slipSkidSmoothConst * (math.Atan2(m.A2, -m.A3) - s.slipSkid)

	// Update Rate of Turn
	if s.gs > 0 && dtw > 0 {
		s.turnRate += uiSmoothConst * ((m.W2*(m.W1-s.w1)-m.W1*(m.W2-s.w2))/(s.gs*s.gs)/dtw - s.turnRate)
	}

	// Update GLoad
	s.gLoad += uiSmoothConst * (-m.A3 - s.gLoad)

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
	if s.staticMode {
		heading = Invalid
	}
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
	if s.staticMode {
		return Invalid
	}
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

// SetConfig lets the user alter some of the configuration settings.
func (s *SimpleState) SetConfig(configMap map[string]float64) {
	uiSmoothConst = configMap["uiSmoothConst"]
	gpsWeight = configMap["gpsWeight"]
	if uiSmoothConst==0 {
		// This doesn't make sense, means user hasn't set correctly
		// Set sensible defaults
		uiSmoothConst = uiSmoothConstDefault
		gpsWeight = gpsWeightDefault
	}
}

func updateLogMap(s *SimpleState, m *Measurement, p map[string]interface{}) {
	var simpleLogMap = map[string]func(s *SimpleState, m *Measurement) float64{
		"Ta":          func(s *SimpleState, m *Measurement) float64 { return s.T },
		"TWa":         func(s *SimpleState, m *Measurement) float64 { return s.TW },
		"Roll":        func(s *SimpleState, m *Measurement) float64 { return s.roll / Deg },
		"Pitch":       func(s *SimpleState, m *Measurement) float64 { return s.pitch / Deg },
		"Heading":     func(s *SimpleState, m *Measurement) float64 { return s.heading / Deg },
		"RollGPS":     func(s *SimpleState, m *Measurement) float64 { return s.rollGPS / Deg },
		"PitchGPS":    func(s *SimpleState, m *Measurement) float64 { return s.pitchGPS / Deg },
		"HeadingGPS":  func(s *SimpleState, m *Measurement) float64 { return s.headingGPS / Deg },
		"GroundSpeed": func(s *SimpleState, m *Measurement) float64 { return s.gs },
		"SmoothW1":    func(s *SimpleState, m *Measurement) float64 { return s.smoothW1 },
		"SmoothW2":    func(s *SimpleState, m *Measurement) float64 { return s.smoothW2 },
		"SmoothGroundSpeed": func(s *SimpleState, m *Measurement) float64 { return s.smoothGS },
		"W1a":         func(s *SimpleState, m *Measurement) float64 { return s.w1 },
		"W2a":         func(s *SimpleState, m *Measurement) float64 { return s.w2 },
		"W3a":         func(s *SimpleState, m *Measurement) float64 { return s.w3 },
		"WValid": func(s *SimpleState, m *Measurement) float64 {
			if m.WValid {
				return 1
			}
			return 0
		},
		"T":          func(s *SimpleState, m *Measurement) float64 { return m.T },
		"TW":         func(s *SimpleState, m *Measurement) float64 { return m.TW },
		"W1":         func(s *SimpleState, m *Measurement) float64 { return m.W1 },
		"W2":         func(s *SimpleState, m *Measurement) float64 { return m.W2 },
		"W3":         func(s *SimpleState, m *Measurement) float64 { return m.W3 },
		"E0":         func(s *SimpleState, m *Measurement) float64 { return s.E0 },
		"E1":         func(s *SimpleState, m *Measurement) float64 { return s.E1 },
		"E2":         func(s *SimpleState, m *Measurement) float64 { return s.E2 },
		"E3":         func(s *SimpleState, m *Measurement) float64 { return s.E3 },
		"EGPS0":      func(s *SimpleState, m *Measurement) float64 { return s.EGPS0 },
		"EGPS1":      func(s *SimpleState, m *Measurement) float64 { return s.EGPS1 },
		"EGPS2":      func(s *SimpleState, m *Measurement) float64 { return s.EGPS2 },
		"EGPS3":      func(s *SimpleState, m *Measurement) float64 { return s.EGPS3 },
		"A1":         func(s *SimpleState, m *Measurement) float64 { return m.A1 },
		"A2":         func(s *SimpleState, m *Measurement) float64 { return m.A2 },
		"A3":         func(s *SimpleState, m *Measurement) float64 { return m.A3 },
		"B1":         func(s *SimpleState, m *Measurement) float64 { return m.B1 },
		"B2":         func(s *SimpleState, m *Measurement) float64 { return m.B2 },
		"B3":         func(s *SimpleState, m *Measurement) float64 { return m.B3 },
		"M1":         func(s *SimpleState, m *Measurement) float64 { return m.M1 },
		"M2":         func(s *SimpleState, m *Measurement) float64 { return m.M2 },
		"M3":         func(s *SimpleState, m *Measurement) float64 { return m.M3 },
		"D1":         func(s *SimpleState, m *Measurement) float64 { return s.D1 },
		"D2":         func(s *SimpleState, m *Measurement) float64 { return s.D2 },
		"D3":         func(s *SimpleState, m *Measurement) float64 { return s.D3 },
		"headingMag": func(s *SimpleState, m *Measurement) float64 { return s.headingMag },
		"slipSkid":   func(s *SimpleState, m *Measurement) float64 { return s.slipSkid },
		"gLoad":      func(s *SimpleState, m *Measurement) float64 { return s.gLoad },
		"turnRate":   func(s *SimpleState, m *Measurement) float64 { return s.turnRate },
		"needsInitialization": func(s *SimpleState, m *Measurement) float64 {
			if s.needsInitialization {
				return 1
			}
			return 0
		},
		"staticMode": func(s *SimpleState, m *Measurement) float64 {
			if s.staticMode {
				return 1
			}
			return 0
		},
		"headingValid": func(s *SimpleState, m *Measurement) float64 {
			if s.headingValid {
				return 1
			}
			return 0
		},
	}

	for k := range simpleLogMap {
		p[k] = simpleLogMap[k](s, m)
	}
}

var SimpleJSONConfig = `
{
  "State": [
    {"pred": "RollGPS", "updt": "Roll", "baseline": 0},
    {"pred": "PitchGPS", "updt": "Pitch", "baseline": 0},
    {"pred": "HeadingGPS", "updt": "Heading"},
    {"updt": "turnRate", "baseline": 0},
    {"updt": "gLoad", "baseline": 1},
    {"updt": "slipSkid", "baseline": 0},
    {"updt": "GroundSpeed", "baseline": 0},
    {"updt": "T"},
    {"pred": "EGPS0", "updt": "E0"},
    {"pred": "EGPS1", "updt": "E1"},
    {"pred": "EGPS2", "updt": "E2"},
    {"pred": "EGPS3", "updt": "E3"},
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
    {"meas": "B3", "baseline": 0}
  ]
}
`
