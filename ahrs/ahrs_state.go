package ahrs

import (
	"math"

	"github.com/skelterjohn/go.matrix"
)

// State holds the complete information describing the state of the aircraft.
// Aircraft frame is non-inertial: 1 is to nose; 2 is to left wing; 3 is up.
// Earth frame is inertial: 1 is east; 2 is north; 3 is up.
// Sensor frame is fixed within aircraft frame, so non-inertial, rotated.
type State struct {
	U1, U2, U3     float64 // Vector for airspeed, aircraft frame, kt
	Z1, Z2, Z3     float64 // Vector for rate of change of airspeed, aircraft frame, G
	E0, E1, E2, E3 float64 // Quaternion rotating aircraft frame to earth frame
	H1, H2, H3     float64 // Vector for gyro rates, earth frame, °/s
	N1, N2, N3     float64 // Vector for earth's magnetic field, earth (inertial) frame, µT

	V1, V2, V3     float64 // (Bias) Vector for windspeed, earth frame, kt
	C1, C2, C3     float64 // Bias vector for accelerometer, sensor frame, G
	F0, F1, F2, F3 float64 // (Bias) quaternion rotating aircraft frame to sensor frame
	D1, D2, D3     float64 // Bias vector for gyro rates, sensor frame, °/s
	L1, L2, L3     float64 // Bias vector for magnetometer direction, sensor frame, µT

	T float64 // Time when state last updated

	M *matrix.DenseMatrix // Covariance matrix of state uncertainty, same order as above vars:
	N *matrix.DenseMatrix // Covariance matrix of state noise per unit time
	// U, Z, E, H, N,
	// V, C, F, D, L

	e11, e12, e13 float64 // cached earth-aircraft rotation matrix
	e21, e22, e23 float64
	e31, e32, e33 float64

	f11, f12, f13 float64 // cached sensor-aircraft rotation matrix
	f21, f22, f23 float64
	f31, f32, f33 float64

	roll, pitch, heading float64                // Fused attitude, Rad
	headingMag           float64                // Magnetic heading, Rad (smoothed)
	slipSkid             float64                // Slip/Skid Angle, Rad (smoothed)
	gLoad                float64                // G Load, G vertical (smoothed)
	turnRate             float64                // turn rate, Rad/s (smoothed)
	needsInitialization  bool                   // Rather than computing, initialize
	aNorm                float64                // Normalization constant by which to scale measured accelerations
	logMap               map[string]interface{} // Map only for analysis/debugging
}

// RollPitchHeading returns the current attitude values as estimated by the Kalman algorithm.
func (s *State) RollPitchHeading() (roll float64, pitch float64, heading float64) {
	roll, pitch, heading = FromQuaternion(s.E0, s.E1, s.E2, s.E3)
	return
}

func (s *State) RollPitchHeadingUncertainty() (droll float64, dpitch float64, dheading float64) {
	droll, dpitch, dheading = VarFromQuaternion(s.E0, s.E1, s.E2, s.E3,
		math.Sqrt(s.M.Get(6, 6)), math.Sqrt(s.M.Get(7, 7)),
		math.Sqrt(s.M.Get(8, 8)), math.Sqrt(s.M.Get(9, 9)))
	return
}

// MagHeading returns the magnetic heading in degrees.
func (s *State) MagHeading() (hdg float64) {
	return s.headingMag / Deg
}

// SlipSkid returns the slip/skid angle in degrees.
func (s *State) SlipSkid() (slipSkid float64) {
	return s.slipSkid / Deg
}

// RateOfTurn returns the turn rate in degrees per second.
func (s *State) RateOfTurn() (turnRate float64) {
	return s.turnRate / Deg
}

// GLoad returns the current G load, in G's.
func (s *State) GLoad() (gLoad float64) {
	return s.gLoad
}

// SetSensorQuaternion changes the AHRS algorithm's sensor quaternion F.
func (s *State) SetSensorQuaternion(f *[4]float64) {
	s.F0 = f[0]
	s.F1 = f[1]
	s.F2 = f[2]
	s.F3 = f[3]
	s.calcRotationMatrices()
}

// GetSensorQuaternion returns the AHRS algorithm's sensor quaternion F.
func (s *State) GetSensorQuaternion() *[4]float64 {
	return &[4]float64{s.F0, s.F1, s.F2, s.F3}
}

// SetCalibrations sets the AHRS accelerometer calibrations to c and gyro calibrations to d.
func (s *State) SetCalibrations(c, d *[3]float64) {
	if c != nil {
		aNorm := math.Sqrt(c[0]*c[0] + c[1]*c[1] + c[2]*c[2])
		if aNorm > 0.5 {
			s.C1 = c[0]
			s.C2 = c[1]
			s.C3 = c[2]
			s.aNorm = math.Sqrt(c[0]*c[0] + c[1]*c[1] + c[2]*c[2])
		} else {
			s.aNorm = 1
		}
	}
	if d != nil {
		s.D1 = d[0]
		s.D2 = d[1]
		s.D3 = d[2]
	}
}

// GetCalibrations sets the AHRS accelerometer calibrations to c and gyro calibrations to d.
func (s *State) GetCalibrations() (c, d *[3]float64) {
	return &[3]float64{s.C1, s.C2, s.C3}, &[3]float64{s.D1, s.D2, s.D3}
}

// SetConfig lets the user alter some of the configuration settings.
func (s *State) SetConfig(configMap map[string]float64) {
	return
}

// Valid returns whether the current state is a valid estimate or if something went wrong in the calculation.
func (s *State) Valid() (ok bool) {
	return true
}

// init puts the algorithm into a known state, on startup or after a reset.
func (s *State) init(m *Measurement) {
	s.needsInitialization = false
	s.T = m.T

	s.roll = 0
	s.pitch = 0
	s.heading = 0

	s.E0, s.E1, s.E2, s.E3 = 1, 0, 0, 0
	s.normalize()

	_, a2, a3 := s.rotateByF(-m.A1, -m.A2, -m.A3)
	_, _, b3 := s.rotateByF(m.B1-s.D1, m.B2-s.D2, m.B3-s.D3)
	m1, m2, _ := s.rotateByF(m.M1, m.M2, m.M3)

	// Initialize Magnetic Heading, Slip/Skid, Rate of Turn, and GLoad.
	_, _, s.headingMag = Regularize(0, 0, math.Atan2(m1, -m2))
	s.slipSkid = math.Atan2(a2, -a3)
	s.turnRate = b3 * Deg
	s.gLoad = -a3 / s.aNorm

	s.updateLogMap(m, s.logMap)
}

// Reset restarts the algorithm from scratch.
func (s *State) Reset() {
	s.needsInitialization = true
}

// GetState returns the state of the system
func (s *State) GetState() *State {
	return s
}

// GetLogMap returns a map providing current state and measurement values for analysis
func (s *State) GetLogMap() (p map[string]interface{}) {
	return s.logMap
}

func (s *State) updateLogMap(m *Measurement, p map[string]interface{}) {
	var logMap = map[string]func(s *State, m *Measurement) float64{
		"Ta":      func(s *State, m *Measurement) float64 { return s.T },
		"Roll":    func(s *State, m *Measurement) float64 { return s.roll / Deg },
		"Pitch":   func(s *State, m *Measurement) float64 { return s.pitch / Deg },
		"Heading": func(s *State, m *Measurement) float64 { return s.heading / Deg },
		"WValid": func(s *State, m *Measurement) float64 {
			if m.WValid {
				return 1
			}
			return 0
		},
		"T":          func(s *State, m *Measurement) float64 { return m.T },
		"TW":         func(s *State, m *Measurement) float64 { return m.TW },
		"W1":         func(s *State, m *Measurement) float64 { return m.W1 },
		"W2":         func(s *State, m *Measurement) float64 { return m.W2 },
		"W3":         func(s *State, m *Measurement) float64 { return m.W3 },
		"E0":         func(s *State, m *Measurement) float64 { return s.E0 },
		"E1":         func(s *State, m *Measurement) float64 { return s.E1 },
		"E2":         func(s *State, m *Measurement) float64 { return s.E2 },
		"E3":         func(s *State, m *Measurement) float64 { return s.E3 },
		"F0":         func(s *State, m *Measurement) float64 { return s.F0 },
		"F1":         func(s *State, m *Measurement) float64 { return s.F1 },
		"F2":         func(s *State, m *Measurement) float64 { return s.F2 },
		"F3":         func(s *State, m *Measurement) float64 { return s.F3 },
		"Z1":         func(s *State, m *Measurement) float64 { return s.Z1 },
		"Z2":         func(s *State, m *Measurement) float64 { return s.Z2 },
		"Z3":         func(s *State, m *Measurement) float64 { return s.Z3 },
		"C1":         func(s *State, m *Measurement) float64 { return s.C1 },
		"C2":         func(s *State, m *Measurement) float64 { return s.C2 },
		"C3":         func(s *State, m *Measurement) float64 { return s.C3 },
		"A1":         func(s *State, m *Measurement) float64 { return m.A1 },
		"A2":         func(s *State, m *Measurement) float64 { return m.A2 },
		"A3":         func(s *State, m *Measurement) float64 { return m.A3 },
		"H1":         func(s *State, m *Measurement) float64 { return s.H1 },
		"H2":         func(s *State, m *Measurement) float64 { return s.H2 },
		"H3":         func(s *State, m *Measurement) float64 { return s.H3 },
		"D1":         func(s *State, m *Measurement) float64 { return s.D1 },
		"D2":         func(s *State, m *Measurement) float64 { return s.D2 },
		"D3":         func(s *State, m *Measurement) float64 { return s.D3 },
		"B1":         func(s *State, m *Measurement) float64 { return m.B1 },
		"B2":         func(s *State, m *Measurement) float64 { return m.B2 },
		"B3":         func(s *State, m *Measurement) float64 { return m.B3 },
		"M1":         func(s *State, m *Measurement) float64 { return m.M1 },
		"M2":         func(s *State, m *Measurement) float64 { return m.M2 },
		"M3":         func(s *State, m *Measurement) float64 { return m.M3 },
		"headingMag": func(s *State, m *Measurement) float64 { return s.headingMag },
		"slipSkid":   func(s *State, m *Measurement) float64 { return s.slipSkid },
		"gLoad":      func(s *State, m *Measurement) float64 { return s.gLoad },
		"turnRate":   func(s *State, m *Measurement) float64 { return s.turnRate },
		"needsInitialization": func(s *State, m *Measurement) float64 {
			if s.needsInitialization {
				return 1
			}
			return 0
		},
	}

	for k := range logMap {
		p[k] = logMap[k](s, m)
	}
}

// normalize normalizes the E & F quaternions in State s to unit magnitude
func (s *State) normalize() {
	ee := math.Sqrt(s.E0*s.E0 + s.E1*s.E1 + s.E2*s.E2 + s.E3*s.E3)
	s.E0 /= ee
	s.E1 /= ee
	s.E2 /= ee
	s.E3 /= ee

	ff := math.Sqrt(s.F0*s.F0 + s.F1*s.F1 + s.F2*s.F2 + s.F3*s.F3)
	s.F0 /= ff
	s.F1 /= ff
	s.F2 /= ff
	s.F3 /= ff

	s.calcRotationMatrices()
}

// calcRotationMatrices populates the rotation matrices in the State based on
// the quaternions E* and F*
func (s *State) calcRotationMatrices() {
	// eij rotates from earth frame j component to aircraft frame i component
	// X_e = E*X_a*conj(E)
	s.e11 = +s.E0*s.E0 + s.E1*s.E1 - s.E2*s.E2 - s.E3*s.E3
	s.e12 = 2 * (-s.E0*s.E3 + s.E1*s.E2)
	s.e13 = 2 * (+s.E0*s.E2 + s.E1*s.E3)
	s.e21 = 2 * (+s.E0*s.E3 + s.E2*s.E1)
	s.e22 = +s.E0*s.E0 - s.E1*s.E1 + s.E2*s.E2 - s.E3*s.E3
	s.e23 = 2 * (-s.E0*s.E1 + s.E2*s.E3)
	s.e31 = 2 * (-s.E0*s.E2 + s.E3*s.E1)
	s.e32 = 2 * (+s.E0*s.E1 + s.E3*s.E2)
	s.e33 = +s.E0*s.E0 - s.E1*s.E1 - s.E2*s.E2 + s.E3*s.E3

	// fij rotates sensor frame j component into aircraft frame i component
	// X_s = F*X_a*conj(F)
	s.f11 = (+s.F0*s.F0 + s.F1*s.F1 - s.F2*s.F2 - s.F3*s.F3)
	s.f12 = 2 * (-s.F0*s.F3 + s.F1*s.F2)
	s.f13 = 2 * (+s.F0*s.F2 + s.F3*s.F1)
	s.f21 = 2 * (+s.F0*s.F3 + s.F1*s.F2)
	s.f22 = (+s.F0*s.F0 - s.F1*s.F1 + s.F2*s.F2 - s.F3*s.F3)
	s.f23 = 2 * (-s.F0*s.F1 + s.F2*s.F3)
	s.f31 = 2 * (-s.F0*s.F2 + s.F3*s.F1)
	s.f32 = 2 * (+s.F0*s.F1 + s.F2*s.F3)
	s.f33 = (+s.F0*s.F0 - s.F1*s.F1 - s.F2*s.F2 + s.F3*s.F3)
}

// rotateByF rotates the input vector by the state's rotation matrix ff.
func (s *State) rotateByF(a1, a2, a3 float64) (z1, z2, z3 float64) {
	z1 = s.f11*a1 + s.f12*a2 + s.f13*a3
	z2 = s.f21*a1 + s.f22*a2 + s.f23*a3
	z3 = s.f31*a1 + s.f32*a2 + s.f33*a3
	return
}

// RollPitchHeading returns the current roll, pitch and heading estimates
// for the State, in degrees
func (s *State) CalcRollPitchHeading() (roll float64, pitch float64, heading float64) {
	roll, pitch, heading = FromQuaternion(s.E0, s.E1, s.E2, s.E3)
	return roll / Deg, pitch / Deg, heading / Deg
}
