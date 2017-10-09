/*
Package ahrs implements a Kalman filter for determining aircraft kinematic state
based on gyro (H) only
* Stage 0: gyro + accel in one dimension
Stage 1: gyro + accel in three dimensions
Stage 2: gyro + accel + magnetometer
Stage 3: gyro + accel + magnetometer + GPS/baro
Stage 4: gyro + accel + magnetometer + GPS/baro + airspeed

E is a quaternion translating from aircraft frame to earth frame (i.e. E_{ea}).
H is the gyro rates in the aircraft frame
D is the estimate of the gyro sensor bias

Equations of Motion
E -> E + 0.5*E*H*dt
H -> H
D -> D

s.E0 += -0.5*dt*s.E1*s.H1*Deg
s.E1 += +0.5*dt*s.E0*s.H1*Deg

s.H1 += 0
s.D1 += 0

Measurement Predictions
m.A2 = 2*s.E0*s.E1
m.A3 = s.E0*s.E0 - s.E1*s.E1

m.B1 = s.H1 + s.D1
*/

package ahrs

import (
	"log"
	"math"

	"fmt"
	"github.com/skelterjohn/go.matrix"
)

type Kalman0State struct {
	State
	f     *matrix.DenseMatrix
	z     *Measurement
	y     *matrix.DenseMatrix
	h     *matrix.DenseMatrix
	ss    *matrix.DenseMatrix
	kk    *matrix.DenseMatrix
}

// Initialize the state at the start of the Kalman filter, based on current measurements
func NewKalman0AHRS() (s *Kalman0State) {
	s = new(Kalman0State)
	s.needsInitialization = true
	s.aNorm = 1
	s.E0 = 1 // Initial guess is East
	s.F0 = 1 // Initial guess is that it's oriented pointing forward and level
	s.normalize()
	s.M = matrix.Zeros(32, 32)
	s.N = matrix.Zeros(32, 32)
	s.f = matrix.Eye(32)
	s.z = NewMeasurement()
	s.y = matrix.Zeros(15, 1)
	s.h = matrix.Zeros(15, 32)
	s.ss = matrix.Zeros(32, 15)
	s.kk = matrix.Zeros(32, 15)
	s.logMap = make(map[string]interface{})
	s.updateLogMap(NewMeasurement(), s.logMap)

	s.gLoad = 1
	return
}

func (s *Kalman0State) init(m *Measurement) {
	s.needsInitialization = false

	s.E0, s.E1, s.E2, s.E3 = 1, 0, 0, 0 // Initial guess is East
	s.F0, s.F1, s.F2, s.F3 = 1, 0, 0, 0
	s.normalize()

	s.T = m.T

	// Diagonal matrix of initial state uncertainties, will be squared into covariance below
	// Specifics here aren't too important--it will change very quickly
	s.M = matrix.Diagonal([]float64{
		Big, Big, Big, // U*3
		Big, Big, Big, // Z*3
		1, 1, Big, Big, // E*4
		2, Big, Big, // H*3
		Big, Big, Big, // N*3
		Big, Big, Big, // V*3
		Big, Big, Big, // C*3
		Big, Big, Big, Big, // F*4
		2, Big, Big, // D*3
		Big, Big, Big, // L*3
	})
	s.M = matrix.Product(s.M, s.M)

	// Diagonal matrix of state process uncertainties per s, will be squared into covariance below
	// Tuning these is more important
	tt := math.Sqrt(60.0 * 60.0) // One-hour time constant for drift of biases V, C, F, D, L
	s.N = matrix.Diagonal([]float64{
		Big, Big, Big, // U*3
		Big, Big, Big, // Z*3
		0.05, 0.05, Big, Big, // E*4
		50, Big, Big, // H*3
		Big, Big, Big, // N*3
		Big, Big, Big, // V*3
		Big, Big, Big, // C*3
		Big, Big, Big, Big, // F*4
		0.1 / tt, Big, Big, // D*3
		Big, Big, Big, // L*3
	})
	s.N = matrix.Product(s.N, s.N)

	s.updateLogMap(m, s.logMap)

	log.Println("Kalman0 Initialized")
	return
}

// Compute runs first the prediction and then the update phases of the Kalman filter
func (s *Kalman0State) Compute(m *Measurement) {
	m.A1, m.A2, m.A3 = s.rotateByF(m.A1, m.A2, m.A3, false)
	m.B1, m.B2, m.B3 = s.rotateByF(m.B1, m.B2, m.B3, false)

	if s.needsInitialization {
		s.init(m)
		return
	}

	s.predict(m.T)
	s.update(m)

	s.updateLogMap(m, s.logMap)
}

// predict performs the prediction phase of the Kalman filter
func (s *Kalman0State) predict(t float64) {
	dt := t - s.T

	// State vectors H and D are unchanged; only E evolves.
	s.E0, s.E1, s.E2, s.E3 = QuaternionRotate(s.E0, s.E1, s.E2, s.E3, s.H1*dt*Deg, 0, 0)
	s.T = t

	s.calcJacobianState(t)
	s.M = matrix.Sum(matrix.Product(s.f, matrix.Product(s.M, s.f.Transpose())), matrix.Scaled(s.N, dt))
}

// predictMeasurement returns the measurement expected given the current state.
func (s *Kalman0State) predictMeasurement() (m *Measurement) {
	m = NewMeasurement()

	m.SValid = true
	_, m.A2, m.A3 = s.rotateByE(0, 0, 1, true)
	m.B1 = s.H1 + s.D1
	m.T = s.T
	return
}

// update applies the Kalman filter corrections given the measurements
func (s *Kalman0State) update(m *Measurement) {
	s.z = s.predictMeasurement()

	s.y.Set(7, 0, m.A2-s.z.A2)
	s.y.Set(8, 0, m.A3-s.z.A3)
	s.y.Set(9, 0, m.B1-s.z.B1)

	s.calcJacobianMeasurement()

	var v float64
	_, _, v = m.Accums[7](m.A2)
	m.M.Set(7, 7, v)
	_, _, v = m.Accums[8](m.A3)
	m.M.Set(8, 8, v)
	_, _, v = m.Accums[9](m.B1)
	m.M.Set(9, 9, v)

	s.ss = matrix.Sum(matrix.Product(s.h, matrix.Product(s.M, s.h.Transpose())), m.M)

	m2, err := s.ss.Inverse()
	if err != nil {
		log.Println("AHRS: Can't invert Kalman gain matrix")
		log.Printf("ss: %s\n", s.ss)
		return
	}
	s.kk = matrix.Product(s.M, matrix.Product(s.h.Transpose(), m2))
	su := matrix.Product(s.kk, s.y)
	s.E0 += su.Get(6, 0)
	s.E1 += su.Get(7, 0)
	s.H1 += su.Get(10, 0)
	s.D1 += su.Get(26, 0)
	s.T = m.T
	s.M = matrix.Product(matrix.Difference(matrix.Eye(32), matrix.Product(s.kk, s.h)), s.M)
	s.normalize()
}

func (s *Kalman0State) calcJacobianState(t float64) {
	dt := t - s.T

	// U*3, Z*3, E*4, H*3, N*3,
	// V*3, C*3, F*4, D*3, L*3

	// s.E0 += -0.5*dt*s.E1*s.H1*Deg
	s.f.Set(6, 7, -0.5*dt*s.H1*Deg) // E0/E1
	s.f.Set(6, 10, -0.5*dt*s.E1*Deg) // E0/H1

	//s.E1 += +0.5*dt*s.E0*s.H1*Deg
	s.f.Set(7, 6, 0.5*dt*s.H1*Deg) // E1/E0
	s.f.Set(7, 10, 0.5*dt*s.E0*Deg) // E1/H1

	// H and D are unchanged.

	return
}

func (s *Kalman0State) calcJacobianMeasurement() {

	// U*3, Z*3, E*4, H*3, N*3,
	// V*3, C*3, F*4, D*3, L*3
	// U*3, W*3, A*3, B*3, M*3

	// m.A2 = 2*s.E0*s.E1
	s.h.Set(7, 6, 2*s.E1) // A2/E0
	s.h.Set(7, 7, 2*s.E0) // A2/E1

	// m.A3 = s.E0*s.E0 - s.E1*s.E1
	s.h.Set(8, 6, +2*s.E0) // A3/E0
	s.h.Set(8, 7, -2*s.E1) // A3/E1

	// m.B1 = s.H1 + s.D1
	s.h.Set(9, 10, 1) // B1/H1
	s.h.Set(9, 26, 1) // B1/D1

	return
}

// SetCalibrations sets the AHRS accelerometer calibrations to c and gyro calibrations to d.
func (s *Kalman0State) SetCalibrations(c, d *[3]float64) {
	return
}

func (s *Kalman0State) updateLogMap(m *Measurement, p map[string]interface{}) {
	s.State.updateLogMap(m, s.logMap)

	/*
	rv, pv, hv := s.State.RollPitchHeadingUncertainty()
	p["PitchVar"] = pv / Deg
	*/

	for k, v := range map[string]*matrix.DenseMatrix {
		"M": s.M,   // M is the state uncertainty covariance matrix
		"N": s.N,   // N is the process uncertainty covariance matrix
		"f": s.f,   // f is the State Jacobian
		"y": s.y,   // y is the correction between actual and predicted measurements
		"h": s.h,   // h is
		"ss": s.ss, // ss is
		"kk": s.kk, // kk is
	} {
		r, c := v.GetSize()
		for i := 0; i < r; i++ {
			for j := 0; j < c; j++ {
				p[fmt.Sprintf("%s[%02d_%02d]", k, i, j)] = v.Get(i, j)
			}
		}
	}

	// z is the predicted measurement
	p["zA1"] = s.z.A1
	p["zB2"] = s.z.B2

}

var Kalman0JSONConfig = `{
  "State": [
    ["Pitch", "PitchActual", 0],
    ["T", null, null],
    ["E0", "E0Actual", null],
    ["E1", "E1Actual", null],
    ["H2", "H2Actual", 0],
    ["D2", "D2Actual", 0],
  ],
  "Measurement": [
    ["A1", null, 0],
    ["B2", null, 0],
  ]
}`
