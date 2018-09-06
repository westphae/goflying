package ahrs

import (
	"fmt"
	"math"

	"github.com/skelterjohn/go.matrix"
)

const (
	Pi              = math.Pi
	G               = 32.1740 / 1.687810 // G is the acceleration due to gravity, kt/s
	Small           = 1e-9
	Big             = 1e9
	Deg             = Pi / 180
	MMDecay         = 1 - 1.0/50 // Exponential decay constant for measurement variances
	Invalid float64 = 3276.7     // 2**15-1
)

// AHRSProvider defines an AHRS (Kalman or other) algorithm, such as ahrs_kalman, ahrs_simple, etc.
type AHRSProvider interface {
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
	// Compute runs both the "predict" and "update" stages of the algorithm, for convenience.
	Compute(m *Measurement)
	// SetSensorQuaternion changes the AHRS algorithm's sensor quaternion F.
	SetSensorQuaternion(f *[4]float64)
	// GetSensorQuaternion returns the AHRS algorithm's sensor quaternion F.
	GetSensorQuaternion() (f *[4]float64)
	// SetCalibrations sets the AHRS accelerometer calibrations to c, gyro calibrations to d,
	// mag scaling to k and mag offset to l.
	SetCalibrations(c, d, k, l *[3]float64)
	// GetCalibrations returns the AHRS accelerometer calibrations c, gyro calibrations d,
	// mag scaling k and mag offset l.
	GetCalibrations() (c, d, k, l *[3]float64)
	// SetConfig allows for configuration of AHRS to be set on the fly, mainly for developers.
	SetConfig(configMap map[string]float64)
	// Valid returns whether the current state is a valid estimate or if something went wrong in the calculation.
	Valid() bool
	// Reset restarts the algorithm from scratch.
	Reset()
	// GetState returns all the information about the current state.
	GetState() *State
	// GetLogMap returns a map customized for each AHRSProvider algorithm to provide more detailed information
	// for debugging and logging.
	GetLogMap() map[string]interface{}
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
func NewMeasurement() (m *Measurement) {
	m = new(Measurement)

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

	return
}

// Regularize ensures that roll, pitch, and heading are in the correct ranges.
// All in radians.
func Regularize(roll, pitch, heading float64) (float64, float64, float64) {
	for pitch > Pi {
		pitch -= 2 * Pi
	}
	for pitch <= -Pi {
		pitch += 2 * Pi
	}
	if pitch > Pi/2 {
		pitch = Pi - pitch
		roll -= Pi
		heading += Pi
	}
	if pitch < -Pi/2 {
		pitch = -Pi - pitch
		roll -= Pi
		heading += Pi
	}

	for roll > Pi {
		roll -= 2 * Pi
	}
	for roll < -Pi {
		roll += 2 * Pi
	}

	for heading >= 2*Pi {
		heading -= 2 * Pi
	}
	for heading < 0 {
		heading += 2 * Pi
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
	for i := 0; i < 3; i++ {
		res[i] = target[i] - f*ortho[i]
	}
	return
}

// MakePerpendicular returns a vector that is perpendicular to both input vectors.
// (Uses the cross product.)
func MakePerpendicular(vec1, vec2 [3]float64) (res *[3]float64, err error) {
	res = new([3]float64)
	for i := 0; i < 3; i++ {
		j := (i + 1) % 3
		k := (i + 2) % 3
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
		diff -= 2 * Pi
	}
	for diff < -Pi {
		diff += 2 * Pi
	}
	return
}
