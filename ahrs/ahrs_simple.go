/*
The idea behind the Simple AHRS algorithm is to use the GPS to compute what the accelerometer should show
and then to create a rotation matrix to map the measured accelerometer vector onto this vector and the
speed vector (GPS Track) onto the sensor x-axis.  The gyro is used to further improve this estimate,
by providing a more instantaneous response than the GPS can provide.
This is really a poor-man's sensor fusion algorithm.  The proper way to do this is with a Kalman Filter,
but this approach is simpler and easier to debug, and should be good enough for most flight conditions.

It is a step on the way to the full Kalman Filter implementation, a bit more obvious what's going on so
the math and stratux integration can be more easily developed and debugged.

This software is protected by the MIT license and is copyright 2017 by Eric Westphal.  I would be happy
for the algorithm to be used elsewhere but please do give me credit if you use either this specific software
or the algorithm behind it.
*/
package ahrs

import (
	"log"
	"math"

	"github.com/skelterjohn/go.matrix"
)

const (
	minDT                      = 1e-6 // Below this time interval, don't recalculate
	maxDT                      = 10.0 // Above this time interval, re-initialize--too stale
	minGS                      = 5.0  // Below this GS, don't use any GPS data
	fastSmoothConstDefault     = 0.7  // Sensible default for fast smoothing of AHRS values
	slowSmoothConstDefault     = 0.1  // Sensible default for slow smoothing of AHRS values
	verySlowSmoothConstDefault = 0.02 // Five-second smoothing mainly for groundspeed, to decide static mode
	gpsWeightDefault           = 0.04 // Sensible default for weight of GPS-derived values in solution
)

var (
	fastSmoothConst     = fastSmoothConstDefault     // Decay constant for smoothing values reported to the user
	slowSmoothConst     = slowSmoothConstDefault     // Decay constant for smoothing values reported to the user
	verySlowSmoothConst = verySlowSmoothConstDefault // Decay constant for smoothing values reported to the user
	gpsWeight           = gpsWeightDefault           // Weight given to GPS quaternion over gyro quaternion
)

type SimpleState struct {
	State
	tW                            float64 // Time of last GPS reading
	eGPS0, eGPS1, eGPS2, eGPS3    float64 // GPS-derived orientation quaternion
	eGyr0, eGyr1, eGyr2, eGyr3    float64 // GPS-derived orientation quaternion
	rollGPS, pitchGPS, headingGPS float64 // GPS/accel-based attitude, Rad
	rollGyr, pitchGyr, headingGyr float64 // Gyro-based attitude, Rad
	w1, w2, w3, gs                float64 // Groundspeed & ROC, Kts
	smoothW1, smoothW2, smoothGS  float64 // Smoothed groundspeed used to determine if stationary
	staticMode                    bool    // For low groundspeed or invalid GPS
	headingValid                  bool    // Whether to slew quickly to correct heading
}

//NewSimpleAHRS returns a new Simple AHRS object.
// It is initialized with a beginning sensor orientation quaternion f0.
func NewSimpleAHRS() (s *SimpleState) {
	s = new(SimpleState)
	s.needsInitialization = true
	s.aNorm = 1
	s.F0 = 1 // Initial guess is that it's oriented pointing forward and level
	s.M = matrix.Zeros(32, 32)
	s.N = matrix.Zeros(32, 32)
	s.logMap = make(map[string]interface{})
	s.updateLogMap(NewMeasurement(), s.logMap)
	return
}

func (s *SimpleState) init(m *Measurement) {
	s.State.init(m)

	s.headingValid = false
	s.tW = m.TW
	if m.WValid {
		s.gs = math.Hypot(m.W1, m.W2)
		s.smoothW1 = s.smoothW1 + verySlowSmoothConst*(m.W1-s.smoothW1)
		s.smoothW2 = s.smoothW2 + verySlowSmoothConst*(m.W2-s.smoothW2)
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

	if s.smoothGS > minGS {
		s.heading = math.Atan2(m.W1, m.W2)
		for s.heading < 0 {
			s.heading += 2 * Pi
		}
		for s.heading >= 2*Pi {
			s.heading -= 2 * Pi
		}
	}

	s.eGPS0, s.eGPS1, s.eGPS2, s.eGPS3 = ToQuaternion(s.roll, s.pitch, s.heading)
	s.eGyr0, s.eGyr1, s.eGyr2, s.eGyr3 = ToQuaternion(s.roll, s.pitch, s.heading)

	s.E0, s.E1, s.E2, s.E3 = s.eGPS0, s.eGPS1, s.eGPS2, s.eGPS3

	s.updateLogMap(m, s.logMap)
}

// Compute performs the AHRSSimple AHRS computations.
func (s *SimpleState) Compute(m *Measurement) {
	if s.needsInitialization {
		s.init(m)
		return
	}
	dt := m.T - s.T
	dtw := m.TW - s.tW

	if dt > maxDT || dtw > maxDT {
		log.Printf("AHRS Info: Reinitializing at %f\n", m.T)
		s.init(m)
		return
	}

	// Rotate measurements from sensor frame to aircraft frame
	a1, a2, a3 := s.rotateByF(-m.A1, -m.A2, -m.A3, false)
	b1, b2, b3 := s.rotateByF(m.B1-s.D1, m.B2-s.D2, m.B3-s.D3, false)
	m1, m2, m3 := s.rotateByF(m.M1, m.M2, m.M3, false)

	// Update estimates of current gyro  and accel rates
	s.Z1 += fastSmoothConst * (a1/s.aNorm - s.Z1)
	s.Z2 += fastSmoothConst * (a2/s.aNorm - s.Z2)
	s.Z3 += fastSmoothConst * (a3/s.aNorm - s.Z3)
	s.H1 += fastSmoothConst * (b1 - s.H1)
	s.H2 += fastSmoothConst * (b2 - s.H2)
	s.H3 += fastSmoothConst * (b3 - s.H3)

	if m.WValid && dtw > minDT {
		s.gs = math.Hypot(m.W1, m.W2)
		s.smoothW1 = s.smoothW1 + verySlowSmoothConst*(m.W1-s.smoothW1)
		s.smoothW2 = s.smoothW2 + verySlowSmoothConst*(m.W2-s.smoothW2)
		s.smoothGS = math.Hypot(s.smoothW1, s.smoothW2)
	}

	ae := [3]float64{0, 0, -1} // Acceleration due to gravity in earth frame
	ve := [3]float64{0, 1, 0}  // Groundspeed in earth frame (default for desktop mode)
	s.staticMode = !(m.WValid && (s.smoothGS > minGS))
	if !s.staticMode {
		if !s.headingValid {
			s.init(m)
			s.headingValid = true
			return
		}
		if dtw < minDT {
			log.Printf("No GPS update at %f\n", m.T)
			return
		}
		ve = [3]float64{m.W1, m.W2, m.W3} // Instantaneous groundspeed in earth frame
		// Instantaneous acceleration in earth frame based on change in GPS groundspeed
		ae[0] -= (m.W1 - s.w1) / dtw / G
		ae[1] -= (m.W2 - s.w2) / dtw / G
		ae[2] -= (m.W3 - s.w3) / dtw / G
	}

	ha, err := MakeUnitVector([3]float64{s.Z1, s.Z2, s.Z3})
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
	e0, e1, e2, e3 = QuaternionSign(e0, e1, e2, e3, s.eGPS0, s.eGPS1, s.eGPS2, s.eGPS3)
	s.eGPS0, s.eGPS1, s.eGPS2, s.eGPS3 = QuaternionNormalize(
		s.eGPS0+fastSmoothConst*(e0-s.eGPS0),
		s.eGPS1+fastSmoothConst*(e1-s.eGPS1),
		s.eGPS2+fastSmoothConst*(e2-s.eGPS2),
		s.eGPS3+fastSmoothConst*(e3-s.eGPS3),
	)

	// By rotating the orientation quaternion at the last time step, s.E, by the measured gyro rates,
	// we get another estimate of the current orientation quaternion using the gyro.
	s.eGyr0, s.eGyr1, s.eGyr2, s.eGyr3 = QuaternionRotate(s.E0, s.E1, s.E2, s.E3, s.H1*dt*Deg, s.H2*dt*Deg, s.H3*dt*Deg)

	// Now fuse the GPS/Accelerometer and Gyro estimates, smooth the result and normalize.
	s.eGPS0, s.eGPS1, s.eGPS2, s.eGPS3 = QuaternionSign(s.eGPS0, s.eGPS1, s.eGPS2, s.eGPS3,
		s.eGyr0, s.eGyr1, s.eGyr2, s.eGyr3)
	de0 := s.eGPS0 - s.eGyr0
	de1 := s.eGPS1 - s.eGyr1
	de2 := s.eGPS2 - s.eGyr2
	de3 := s.eGPS3 - s.eGyr3
	s.E0, s.E1, s.E2, s.E3 = QuaternionNormalize(
		s.eGyr0+gpsWeight*de0*(0.5+de0*de0),
		s.eGyr1+gpsWeight*de1*(0.5+de1*de1),
		s.eGyr2+gpsWeight*de2*(0.5+de2*de2),
		s.eGyr3+gpsWeight*de3*(0.5+de3*de3),
	)

	s.roll, s.pitch, s.heading = FromQuaternion(s.E0, s.E1, s.E2, s.E3)
	s.rollGPS, s.pitchGPS, s.headingGPS = FromQuaternion(s.eGPS0, s.eGPS1, s.eGPS2, s.eGPS3)
	s.rollGyr, s.pitchGyr, s.headingGyr = FromQuaternion(s.eGyr0, s.eGyr1, s.eGyr2, s.eGyr3)

	// Update Magnetic Heading
	dhM := AngleDiff(math.Atan2(m1, -m2), s.headingMag) + 0*m3
	s.headingMag += slowSmoothConst * dhM
	for s.headingMag < 0 {
		s.headingMag += 2 * Pi
	}
	for s.headingMag >= 2*Pi {
		s.headingMag -= 2 * Pi
	}

	// Update Slip/Skid
	s.slipSkid += slowSmoothConst * (math.Atan2(a2, -a3) - s.slipSkid)

	// Update Rate of Turn
	if s.gs > 0 && dtw > 0 {
		s.turnRate += slowSmoothConst * ((m.W2*(m.W1-s.w1)-m.W1*(m.W2-s.w2))/(s.gs*s.gs)/dtw - s.turnRate)
	}

	// Update GLoad
	s.gLoad += slowSmoothConst * (-a3/s.aNorm - s.gLoad)

	s.updateLogMap(m, s.logMap)

	s.T = m.T
	s.tW = m.TW
	s.w1 = m.W1
	s.w2 = m.W2
	s.w3 = m.W3
}

// RollPitchHeading returns the current attitude values as estimated by the Kalman algorithm.
func (s *SimpleState) RollPitchHeading() (roll float64, pitch float64, heading float64) {
	roll, pitch, heading = s.State.RollPitchHeading()
	if s.staticMode {
		heading = Invalid
	}
	return
}

// RateOfTurn returns the turn rate in degrees per second.
func (s *SimpleState) RateOfTurn() (turnRate float64) {
	if s.staticMode {
		return Invalid
	}
	return s.State.RateOfTurn()
}

// SetConfig lets the user alter some of the configuration settings.
func (s *SimpleState) SetConfig(configMap map[string]float64) {
	if v, ok := configMap["fastSmoothConst"]; ok {
		fastSmoothConst = v
	}
	if v, ok := configMap["slowSmoothConst"]; ok {
		slowSmoothConst = v
	}
	if v, ok := configMap["verySlowSmoothConst"]; ok {
		verySlowSmoothConst = v
	}
	if v, ok := configMap["gpsWeight"]; ok {
		gpsWeight = v
	}
	if fastSmoothConst == 0 || slowSmoothConst == 0 || verySlowSmoothConst == 0 {
		// This doesn't make sense, means user hasn't set correctly.
		// Set sensible defaults.
		fastSmoothConst = fastSmoothConstDefault
		slowSmoothConst = slowSmoothConstDefault
		verySlowSmoothConst = verySlowSmoothConstDefault
		gpsWeight = gpsWeightDefault
	}
}

func (s *SimpleState) updateLogMap(m *Measurement, p map[string]interface{}) {
	s.State.updateLogMap(m, s.logMap)
	var simpleLogMap = map[string]func(s *SimpleState, m *Measurement) float64{
		"RollGPS":           func(s *SimpleState, m *Measurement) float64 { return s.rollGPS / Deg },
		"PitchGPS":          func(s *SimpleState, m *Measurement) float64 { return s.pitchGPS / Deg },
		"HeadingGPS":        func(s *SimpleState, m *Measurement) float64 { return s.headingGPS / Deg },
		"RollGyr":           func(s *SimpleState, m *Measurement) float64 { return s.rollGyr / Deg },
		"PitchGyr":          func(s *SimpleState, m *Measurement) float64 { return s.pitchGyr / Deg },
		"HeadingGyr":        func(s *SimpleState, m *Measurement) float64 { return s.headingGyr / Deg },
		"GroundSpeed":       func(s *SimpleState, m *Measurement) float64 { return s.gs },
		"SmoothW1":          func(s *SimpleState, m *Measurement) float64 { return s.smoothW1 },
		"SmoothW2":          func(s *SimpleState, m *Measurement) float64 { return s.smoothW2 },
		"SmoothGroundSpeed": func(s *SimpleState, m *Measurement) float64 { return s.smoothGS },
		"TWa":               func(s *SimpleState, m *Measurement) float64 { return s.tW },
		"W1a":               func(s *SimpleState, m *Measurement) float64 { return s.w1 },
		"W2a":               func(s *SimpleState, m *Measurement) float64 { return s.w2 },
		"W3a":               func(s *SimpleState, m *Measurement) float64 { return s.w3 },
		"EGPS0":             func(s *SimpleState, m *Measurement) float64 { return s.eGPS0 },
		"EGPS1":             func(s *SimpleState, m *Measurement) float64 { return s.eGPS1 },
		"EGPS2":             func(s *SimpleState, m *Measurement) float64 { return s.eGPS2 },
		"EGPS3":             func(s *SimpleState, m *Measurement) float64 { return s.eGPS3 },
		"EGyr0":             func(s *SimpleState, m *Measurement) float64 { return s.eGyr0 },
		"EGyr1":             func(s *SimpleState, m *Measurement) float64 { return s.eGyr1 },
		"EGyr2":             func(s *SimpleState, m *Measurement) float64 { return s.eGyr2 },
		"EGyr3":             func(s *SimpleState, m *Measurement) float64 { return s.eGyr3 },
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

var SimpleJSONConfig = `{
  "State": [
    ["Roll", "RollGPS", "RollGyr", "RollActual", 0],
    ["Pitch", "PitchGPS", "PitchGyr", "PitchActual", 0],
    ["Heading", "HeadingGPS", "HeadingGyr", "HeadingActual", null],
    ["turnRate", null, null, "turnRateActual", 0],
    ["gLoad", null, null, "gLoadActual", 1],
    ["slipSkid", null, null, "slipSkidActual", 0],
    ["GroundSpeed", null, null, null, 0],
    ["T", null, null, null, null],
    ["E0", "EGPS0", "EGyr0", "E0Actual", null],
    ["E1", "EGPS1", "EGyr1", "E1Actual", null],
    ["E2", "EGPS2", "EGyr2", "E2Actual", null],
    ["E3", "EGPS3", "EGyr3", "E3Actual", null],
    ["Z1", null, null, "Z1Actual", 0],
    ["Z2", null, null, "Z2Actual", 0],
    ["Z3", null, null, "Z3Actual", -1]
    ["C1", null, null, "C1Actual", 0],
    ["C2", null, null, "C2Actual", 0],
    ["C3", null, null, "C3Actual", 0]
    ["H1", null, null, "H1Actual", 0],
    ["H2", null, null, "H2Actual", 0],
    ["H3", null, null, "H3Actual", 0]
    ["D1", null, null, "D1Actual", 0],
    ["D2", null, null, "D2Actual", 0],
    ["D3", null, null, "D3Actual", 0]
  ],
  "Measurement": [
    ["W1", "W1a", 0],
    ["W2", "W2a", 0],
    ["W3", "W3a", 0],
    ["A1", null, 0],
    ["A2", null, 0],
    ["A3", null, 0],
    ["B1", null, 0],
    ["B2", null, 0],
    ["B3", null, 0]
    ["M1", null, 0],
    ["M2", null, 0],
    ["M3", null, 0]
  ]
}`
