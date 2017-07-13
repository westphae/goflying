package ahrs

import (
	"fmt"
	"math"

	"github.com/skelterjohn/go.matrix"
)

const (
	Pi      = math.Pi
	G       = 32.1740 / 1.687810 // G is the acceleration due to gravity, kt/s
	Small   = 1e-9
	Big     = 1e9
	Deg     = Pi / 180
	MMDecay = 1 - 1.0/50 // Exponential decay constant for measurement variances
	Invalid float64  = 3276.7 // 2**15-1
)

// State holds the complete information describing the state of the aircraft
// Aircraft frame is noninertial: 1 is to nose; 2 is to left wing; 3 is up
// Earth frame is inertial: 1 is east; 2 is north; 3 is up
// Sensor frame is fixed within aircraft frame, so noninertial, rotated
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
}

// Measurement holds the measurements used for updating the Kalman filter:
// true airspeed, groundspeed, accelerations, gyro rates, magnetometer, time;
// along with variance accumulators and uncertainty matrix.
// Note: some sensors (e.g. airspeed and magnetometer) may not be available
// until appropriate sensors are working.
type Measurement struct { // Order here also defines order in the matrices below
	UValid, WValid, SValid, MValid bool // Do we have valid airspeed, GPS, accel/gyro, and magnetometer readings?
	// U, W, A, B, M
	U1, U2, U3 float64 // Vector of measured airspeed, kt, aircraft (accelerated) frame
	W1, W2, W3 float64 // Vector of GPS speed in N/S, E/W and U/D directions, kt, latlong axes, earth (inertial) frame
	A1, A2, A3 float64 // Vector holding accelerometer readings, G, aircraft (accelerated) frame
	B1, B2, B3 float64 // Vector of gyro rates in roll, pitch, heading axes, °/s, aircraft (accelerated) frame
	M1, M2, M3 float64 // Vector of magnetometer readings, µT, aircraft (accelerated) frame
	TW, TU, T  float64 // Timestamp of GPS, airspeed and sensor readings
	//TODO westphae: track separate measurement timestamps for Gyro/Accel, Magnetometer, GPS, Baro

	Accums [15]func(float64) (float64, float64, float64) // Accumulators to track means & variances of all variables

	M *matrix.DenseMatrix // Measurement noise covariance
}

// NewMeasurement returns a pointer to an empty AHRS Measurement.
// Uncertainty matrix and variance accumulators are properly initialized.
func NewMeasurement() *Measurement {
	m := new(Measurement)

	m.M = matrix.Scaled(matrix.Eye(15), Big)

	m.Accums[0] = NewVarianceAccumulator(0, 1, MMDecay)
	m.Accums[1] = NewVarianceAccumulator(0, 1, MMDecay)
	m.Accums[2] = NewVarianceAccumulator(0, 1, MMDecay)
	m.Accums[3] = NewVarianceAccumulator(0, 0.2, MMDecay)
	m.Accums[4] = NewVarianceAccumulator(0, 0.2, MMDecay)
	m.Accums[5] = NewVarianceAccumulator(0, 0.2, MMDecay)
	m.Accums[6] = NewVarianceAccumulator(0, 0.3, MMDecay) // 0.0004 typical from sensor
	m.Accums[7] = NewVarianceAccumulator(0, 0.3, MMDecay)
	m.Accums[8] = NewVarianceAccumulator(0, 0.3, MMDecay)
	m.Accums[9] = NewVarianceAccumulator(0, 1, MMDecay) // 0.02 typical from sensor
	m.Accums[10] = NewVarianceAccumulator(0, 1, MMDecay)
	m.Accums[11] = NewVarianceAccumulator(0, 1, MMDecay)
	m.Accums[12] = NewVarianceAccumulator(0, 80, MMDecay) // 70 typical from sensor
	m.Accums[13] = NewVarianceAccumulator(0, 80, MMDecay)
	m.Accums[14] = NewVarianceAccumulator(0, 80, MMDecay)

	return m
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

// Regularize ensures that roll, pitch, and heading are in the correct ranges.
// All in radians.
func Regularize(roll, pitch, heading float64) (float64, float64, float64) {
	for pitch > Pi {
		pitch -= 2*Pi
	}
	for pitch <= -Pi {
		pitch += 2*Pi
	}
	if pitch > Pi / 2 {
		pitch = Pi - pitch
		roll -= Pi
		heading += Pi
	}
	if pitch < -Pi / 2 {
		pitch = -Pi - pitch
		roll -= Pi
		heading += Pi
	}

	for roll > Pi {
		roll -= 2*Pi
	}
	for roll < -Pi {
		roll += 2*Pi
	}

	for heading >= 2*Pi {
		heading -= 2*Pi
	}
	for heading < 0 {
		heading += 2*Pi
	}
	return roll, pitch, heading
}

// MakeUnitVector re-scales the vector vec into a unit vector.
func MakeUnitVector(vec [3]float64) (res *[3]float64, err error) {
	res = new([3]float64)
	s := math.Sqrt(vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2])
	if s > 0 {
		res[0] = vec[0] / s
		res[1] = vec[1] / s
		res[2] = vec[2] / s
		return res, nil
	}
	return nil, fmt.Errorf("Error: vector is length zero")
}

// MakeOrthogonal returns a vector close to the input target vector
// but with the projection on the input ortho vector removed.
func MakeOrthogonal(target, ortho [3]float64) (res *[3]float64) {
	res = new([3]float64)
	f := target[0]*ortho[0] + target[1]*ortho[1] + target[2]*ortho[2]
	for i:=0; i<3; i++ {
		res[i] = target[i] - f * ortho[i]
	}
	return
}

// MakePerpendicular returns a vector that is perpendicular to both input vectors.
// (Uses the cross product.)
func MakePerpendicular(vec1, vec2 [3]float64) (res *[3]float64, err error) {
	res = new([3]float64)
	for i:=0; i < 3; i++ {
		j := (i+1)%3
		k := (i+2)%3
		res[i] = vec1[j]*vec2[k] - vec1[k]*vec2[j]
	}
	res, err = MakeUnitVector(*res)
	if err != nil {
		return nil, fmt.Errorf("Error: vectors are parallel or length 0")
	}
	return res, nil
}

// MakeHardSoftRotationMatrix constructs a rotation matrix that rotates the unit vector h1 exactly into the unit vector
// h2 and the unit vector s1 as nearly as possible into the unit vector s2.  h1 is "hard-mapped" into h2 and
// s1 is "soft-mapped" into s2.
// It is up to the caller to ensure that all vectors are unit vectors.
func MakeHardSoftRotationMatrix(h1, s1, h2, s2 [3]float64) (rotmat *[3][3]float64, err error) {
	rotmat = new([3][3]float64)
	var (
		v1, v2, w1, w2 *[3]float64
	)

	// The easiest way to "soft-rotate" s1 into s2 is by making s1 orthogonal to h1 and s2 orthogonal to h2
	// and then mapping them exactly.
	v1, err = MakeUnitVector(*MakeOrthogonal(s1, h1))
	if err != nil {
		return nil, err
	}
	v2, err = MakeUnitVector(*MakeOrthogonal(s2, h2))
	if err != nil {
		return nil, err
	}

	// Now construct a third unit vector orthogonal to h1, s1, and the same for h2, s2
	w1, err = MakePerpendicular(h1, s1)
	if err != nil {
		return nil, err
	}
	w2, err = MakePerpendicular(h2, s2)
	if err != nil {
		return nil, err
	}

	// rotmat = [h2 v2 w2] @ [h1 v1 w1].T
	rotmat[0][0] = h2[0]*h1[0] + v2[0]*v1[0] + w2[0]*w1[0]
	rotmat[0][1] = h2[0]*h1[1] + v2[0]*v1[1] + w2[0]*w1[1]
	rotmat[0][2] = h2[0]*h1[2] + v2[0]*v1[2] + w2[0]*w1[2]
	rotmat[1][0] = h2[1]*h1[0] + v2[1]*v1[0] + w2[1]*w1[0]
	rotmat[1][1] = h2[1]*h1[1] + v2[1]*v1[1] + w2[1]*w1[1]
	rotmat[1][2] = h2[1]*h1[2] + v2[1]*v1[2] + w2[1]*w1[2]
	rotmat[2][0] = h2[2]*h1[0] + v2[2]*v1[0] + w2[2]*w1[0]
	rotmat[2][1] = h2[2]*h1[1] + v2[2]*v1[1] + w2[2]*w1[1]
	rotmat[2][2] = h2[2]*h1[2] + v2[2]*v1[2] + w2[2]*w1[2]

	return rotmat, nil
}

func AngleDiff(a, b float64) (diff float64) {
	diff = a - b
	for diff > Pi {
		diff -= 2*Pi
	}
	for diff < -Pi {
		diff += 2*Pi
	}
	return
}

// AHRSProvider defines an AHRS (Kalman or other) algorithm, such as ahrs_kalman, ahrs_simple, etc.
type AHRSProvider interface {
	// Compute runs both the "predict" and "update" stages of the algorithm, for convenience.
	Compute(m *Measurement)
	// SetSensorQuaternion changes the AHRS algorithm's sensor quaternion F.
	SetSensorQuaternion(f *[4]float64)
	// GetSensorQuaternion returns the AHRS algorithm's sensor quaternion F.
	GetSensorQuaternion() (f *[4]float64)
	// SetCalibrations sets the AHRS accelerometer calibrations to c and gyro calibrations to d.
	SetCalibrations(c, d *[3]float64)
	// GetCalibrations returns the AHRS accelerometer calibrations c and gyro calibrations d.
	GetCalibrations() (c, d *[3]float64)
	// SetConfig allows for configuration of AHRS to be set on the fly, mainly for developers.
	SetConfig(configMap map[string]float64)
	// Valid returns whether the current state is a valid estimate or if something went wrong in the calculation.
	Valid() bool
	// Reset restarts the algorithm from scratch.
	Reset()
	// RollPitchHeading returns the current attitude values as estimated by the Kalman algorithm.
	RollPitchHeading() (roll float64, pitch float64, heading float64)
	// MagHeading returns the current magnetic heading in degrees as estimated by the Kalman algorithm.
	MagHeading() (hdg float64)
	// SlipSkid returns the slip/skid angle in degrees as estimated by the Kalman algorithm.
	SlipSkid() (slipSkid float64)
	// RateOfTurn returns the turn rate in degrees per second as estimated by the Kalman algorithm.
	RateOfTurn() (turnRate float64)
	// GLoad returns the current G load, in G's as estimated by the Kalman algorithm.
	GLoad() (gLoad float64)
	// GetState returns all the information about the current state.
	GetState() *State
	// GetLogMap returns a map customized for each AHRSProvider algorithm to provide more detailed information
	// for debugging and logging.
	GetLogMap() map[string]interface{}
}
