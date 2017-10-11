/*
Package ahrs implements a Kalman filter for determining aircraft kinematic state
based on gyro (H) only
* Stage 1: gyro only
(Stage 2: gyro + accel)
(Stage 3: gyro + accel + magnetometer)
(Stage 4: gyro + accel + magnetometer + GPS/baro)
(Stage 5: gyro + accel + magnetometer + GPS/baro + airspeed)

E is a quaternion translating from aircraft frame to earth frame
H is the gyro rates in the aircraft frame
F is a quaternion translating from aircraft frame to sensor frame
D is the estimate of the gyro bias

Equations of Motion
E -> E + 0.5*E*H*dt
H -> H
F -> F
D -> D

s.E0 += 0.5*dt*(-s.E1*s.H1 - s.E2*s.H2 - s.E3*s.H3)*Deg
s.E1 += 0.5*dt*(+s.E0*s.H1 - s.E3*s.H2 + s.E2*s.H3)*Deg
s.E2 += 0.5*dt*(+s.E3*s.H1 + s.E0*s.H2 - s.E1*s.H3)*Deg
s.E3 += 0.5*dt*(-s.E2*s.H1 + s.E1*s.H2 + s.E0*s.H3)*Deg

Measurement Predictions
B = F*H + D
*/

package ahrs

import (
	"github.com/skelterjohn/go.matrix"
	"math"
)

type Kalman1State struct {
	State
}

func (s *Kalman1State) CalcRollPitchHeadingUncertainty() (droll float64, dpitch float64, dheading float64) {
	droll, dpitch, dheading = VarFromQuaternion(s.E0, s.E1, s.E2, s.E3,
		math.Sqrt(s.M.Get(6, 6)), math.Sqrt(s.M.Get(7, 7)),
		math.Sqrt(s.M.Get(8, 8)), math.Sqrt(s.M.Get(9, 9)))
	return
}

// GetState returns the Kalman state of the system
func (s *Kalman1State) GetState() *State {
	return &s.State
}

// GetStateMap returns the state information for analysis
func (s *Kalman1State) GetStateMap() (dat *map[string]float64) {
	phi, theta, psi := FromQuaternion(s.E0, s.E1, s.E2, s.E3)
	phi0, theta0, psi0 := FromQuaternion(s.F0, s.F1, s.F2, s.F3)
	dphi, dtheta, dpsi := s.CalcRollPitchHeadingUncertainty()
	dphi0, dtheta0, dpsi0 := VarFromQuaternion(s.F0, s.F1, s.F2, s.F3,
		math.Sqrt(s.M.Get(22, 22)), math.Sqrt(s.M.Get(23, 23)),
		math.Sqrt(s.M.Get(24, 24)), math.Sqrt(s.M.Get(25, 25)))
	dat = &map[string]float64{
		"T":       s.T,
		"E0":      s.E0,
		"E1":      s.E1,
		"E2":      s.E2,
		"E3":      s.E3,
		"Phi":     phi / Deg,
		"Theta":   theta / Deg,
		"Psi":     psi / Deg,
		"H1":      s.H1,
		"H2":      s.H2,
		"H3":      s.H3,
		"F0":      s.F0,
		"F1":      s.F1,
		"F2":      s.F2,
		"F3":      s.F3,
		"Phi0":    phi0 / Deg,
		"Theta0":  theta0 / Deg,
		"Psi0":    psi0 / Deg,
		"D1":      s.D1,
		"D2":      s.D2,
		"D3":      s.D3,
		"dPhi":    dphi,
		"dTheta":  dtheta,
		"dPsi":    dpsi,
		"dH1":     math.Sqrt(s.M.Get(10, 10)),
		"dH2":     math.Sqrt(s.M.Get(11, 11)),
		"dH3":     math.Sqrt(s.M.Get(12, 12)),
		"dPhi0":   dphi0,
		"dTheta0": dtheta0,
		"dPsi0":   dpsi0,
		"dD1":     math.Sqrt(s.M.Get(26, 26)),
		"dD2":     math.Sqrt(s.M.Get(27, 27)),
		"dD3":     math.Sqrt(s.M.Get(28, 28)),
	}
	return
}

// Initialize the state at the start of the Kalman filter, based on current measurements
func InitializeKalman1(m *Measurement) (s *Kalman1State) {
	s = new(Kalman1State)
	s.init(m)
	return
}

func (s *Kalman1State) init(m *Measurement) {

	s.E0 = 1 // Initial guess is East
	s.F0 = 1 // Initial guess is that it's oriented pointing forward and level
	s.normalize()

	s.T = m.T

	// Diagonal matrix of initial state uncertainties, will be squared into covariance below
	// Specifics here aren't too important--it will change very quickly
	s.M = matrix.Diagonal([]float64{
		Big, Big, Big, // U*3
		Big, Big, Big, // Z*3
		0.5, 0.5, 0.5, 0.5, // E*4
		2, 2, 2, // H*3
		Big, Big, Big, // N*3
		Big, Big, Big, // V*3
		Big, Big, Big, // C*3
		0.002, 0.002, 0.002, 0.002, // F*4
		0.1, 0.1, 0.1, // D*3
		Big, Big, Big, // L*3
	})
	s.M = matrix.Product(s.M, s.M)

	// Diagonal matrix of state process uncertainties per s, will be squared into covariance below
	// Tuning these is more important
	tt := math.Sqrt(60.0 * 60.0) // One-hour time constant for drift of biases V, C, F, D, L
	s.N = matrix.Diagonal([]float64{
		Big, Big, Big, // U*3
		Big, Big, Big, // Z*3
		0.02, 0.02, 0.02, 0.02, // E*4
		1, 1, 1, // H*3
		Big, Big, Big, // N*3
		Big, Big, Big, // V*3
		Big, Big, Big, // C*3
		0.0001 / tt, 0.0001 / tt, 0.0001 / tt, 0.0001 / tt, // F*4
		0.1 / tt, 0.1 / tt, 0.1 / tt, // D*3
		Big, Big, Big, // L*3
	})
	s.N = matrix.Product(s.N, s.N)

	return
}

// Compute runs first the prediction and then the update phases of the Kalman filter
func (s *Kalman1State) Compute(m *Measurement) {
	s.Predict(m.T)
	s.Update(m)
}

// Valid applies some heuristics to detect whether the computed state is valid or not
func (s *Kalman1State) Valid() (ok bool) {
	return true
}

// Predict performs the prediction phase of the Kalman filter
func (s *Kalman1State) Predict(t float64) {
	f := s.calcJacobianState(t)
	dt := t - s.T

	s.E0 += 0.5 * dt * (-s.E1*s.H1 - s.E2*s.H2 - s.E3*s.H3) * Deg
	s.E1 += 0.5 * dt * (+s.E0*s.H1 - s.E3*s.H2 + s.E2*s.H3) * Deg
	s.E2 += 0.5 * dt * (+s.E3*s.H1 + s.E0*s.H2 - s.E1*s.H3) * Deg
	s.E3 += 0.5 * dt * (-s.E2*s.H1 + s.E1*s.H2 + s.E0*s.H3) * Deg
	s.normalize()

	s.T = t

	s.M = matrix.Sum(matrix.Product(f, matrix.Product(s.M, f.Transpose())), matrix.Scaled(s.N, dt))
}

func (s *Kalman1State) PredictMeasurement() (m *Measurement) {
	m = NewMeasurement()

	m.SValid = true
	m.B1 = s.f11*s.H1 + s.f12*s.H2 + s.f13*s.H3 + s.D1
	m.B2 = s.f21*s.H1 + s.f22*s.H2 + s.f23*s.H3 + s.D2
	m.B3 = s.f31*s.H1 + s.f32*s.H2 + s.f33*s.H3 + s.D3

	m.T = s.T

	return
}

// Update applies the Kalman filter corrections given the measurements
func (s *Kalman1State) Update(m *Measurement) {
	z := s.PredictMeasurement()

	y := matrix.Zeros(15, 1)
	y.Set(9, 0, m.B1-z.B1)
	y.Set(10, 0, m.B2-z.B2)
	y.Set(11, 0, m.B3-z.B3)

	h := s.calcJacobianMeasurement()

	var v float64
	_, _, v = m.Accums[9](m.B1)
	m.M.Set(9, 9, v)
	_, _, v = m.Accums[10](m.B2)
	m.M.Set(10, 10, v)
	_, _, v = m.Accums[11](m.B3)
	m.M.Set(11, 11, v)

	ss := matrix.Sum(matrix.Product(h, matrix.Product(s.M, h.Transpose())), m.M)

	m2, err := ss.Inverse()
	if err != nil {
		return
	}
	kk := matrix.Product(s.M, matrix.Product(h.Transpose(), m2))
	su := matrix.Product(kk, y)
	s.E0 += su.Get(6, 0)
	s.E1 += su.Get(7, 0)
	s.E2 += su.Get(8, 0)
	s.E3 += su.Get(9, 0)
	s.H1 += su.Get(10, 0)
	s.H2 += su.Get(11, 0)
	s.H3 += su.Get(12, 0)
	s.F0 += su.Get(22, 0)
	s.F1 += su.Get(23, 0)
	s.F2 += su.Get(24, 0)
	s.F3 += su.Get(25, 0)
	s.D1 += su.Get(26, 0)
	s.D2 += su.Get(27, 0)
	s.D3 += su.Get(28, 0)
	s.T = m.T
	s.M = matrix.Product(matrix.Difference(matrix.Eye(32), matrix.Product(kk, h)), s.M)
	s.normalize()
}

func (s *Kalman1State) calcJacobianState(t float64) (jac *matrix.DenseMatrix) {
	dt := t - s.T

	jac = matrix.Eye(32)
	// U*3, Z*3, E*4, H*3, N*3,
	// V*3, C*3, F*4, D*3, L*3

	//s.E0 += 0.5*dt*(-s.E1*s.H1 - s.E2*s.H2 - s.E3*s.H3)*Deg
	jac.Set(6, 7, -0.5*dt*s.H1*Deg)  // E0/E1
	jac.Set(6, 8, -0.5*dt*s.H2*Deg)  // E0/E2
	jac.Set(6, 9, -0.5*dt*s.H3*Deg)  // E0/E3
	jac.Set(6, 10, -0.5*dt*s.E1*Deg) // E0/H1
	jac.Set(6, 11, -0.5*dt*s.E2*Deg) // E0/H2
	jac.Set(6, 12, -0.5*dt*s.E3*Deg) // E0/H3

	//s.E1 += 0.5*dt*(+s.E0*s.H1 - s.E3*s.H2 + s.E2*s.H3)*Deg
	jac.Set(7, 6, +0.5*dt*s.H1*Deg)  // E1/E0
	jac.Set(7, 8, +0.5*dt*s.H3*Deg)  // E1/E2
	jac.Set(7, 9, -0.5*dt*s.H2*Deg)  // E1/E3
	jac.Set(7, 10, +0.5*dt*s.E0*Deg) // E1/H1
	jac.Set(7, 11, -0.5*dt*s.E3*Deg) // E1/H2
	jac.Set(7, 12, +0.5*dt*s.E2*Deg) // E1/H3

	//s.E2 += 0.5*dt*(+s.E3*s.H1 + s.E0*s.H2 - s.E1*s.H3)*Deg
	jac.Set(8, 6, +0.5*dt*s.H2*Deg)  // E2/E0
	jac.Set(8, 7, -0.5*dt*s.H3*Deg)  // E2/E1
	jac.Set(8, 9, +0.5*dt*s.H1*Deg)  // E2/E3
	jac.Set(8, 10, +0.5*dt*s.E3*Deg) // E2/H1
	jac.Set(8, 11, +0.5*dt*s.E0*Deg) // E2/H2
	jac.Set(8, 12, -0.5*dt*s.E1*Deg) // E2/H3

	//s.E3 += 0.5*dt*(-s.E2*s.H1 + s.E1*s.H2 + s.E0*s.H3)*Deg
	jac.Set(9, 6, +0.5*dt*s.H3*Deg)  // E3/E0
	jac.Set(9, 7, +0.5*dt*s.H2*Deg)  // E3/E1
	jac.Set(9, 8, -0.5*dt*s.H1*Deg)  // E3/E2
	jac.Set(9, 10, -0.5*dt*s.E2*Deg) // E3/H1
	jac.Set(9, 11, +0.5*dt*s.E1*Deg) // E3/H2
	jac.Set(9, 12, +0.5*dt*s.E0*Deg) // E3/H3

	return
}

func (s *Kalman1State) calcJacobianMeasurement() (jac *matrix.DenseMatrix) {
	// B = F*H*conj(F) + D

	jac = matrix.Zeros(15, 32)
	// U*3, Z*3, E*4, H*3, N*3,
	// V*3, C*3, F*4, D*3, L*3
	// U*3, W*3, A*3, B*3, M*3

	//m.B1 = s.f11*s.H1 + s.f12*s.H2 + s.f13*s.H3 + s.D1
	//s.f11 = (+s.F0 * s.F0 + s.F1 * s.F1 - s.F2 * s.F2 - s.F3 * s.F3)
	//s.f12 = 2*(-s.F0 * s.F3 + s.F1 * s.F2)
	//s.f13 = 2*(+s.F0 * s.F2 + s.F3 * s.F1)
	jac.Set(9, 10, s.f11)                              // B1/H1
	jac.Set(9, 11, s.f12)                              // B1/H2
	jac.Set(9, 12, s.f13)                              // B1/H3
	jac.Set(9, 22, 2*(s.H1*s.F0-s.H2*s.F3+s.H3*s.F2))  // B1/F0
	jac.Set(9, 23, 2*(s.H1*s.F1+s.H2*s.F2+s.H3*s.F3))  // B1/F1
	jac.Set(9, 24, 2*(-s.H1*s.F2+s.H2*s.F1+s.H3*s.F0)) // B1/F2
	jac.Set(9, 25, 2*(-s.H1*s.F3-s.H2*s.F0+s.H3*s.F1)) // B1/F3
	jac.Set(9, 26, 1)                                  // B1/D1

	//m.B2 = s.f21*s.H1 + s.f22*s.H2 + s.f23*s.H3 + s.D2
	//s.f21 = 2*(+s.F0 * s.F3 + s.F1 * s.F2)
	//s.f22 = (+s.F0 * s.F0 - s.F1 * s.F1 + s.F2 * s.F2 - s.F3 * s.F3)
	//s.f23 = 2*(-s.F0 * s.F1 + s.F2 * s.F3)
	jac.Set(10, 10, s.f21)                             // B2/H1
	jac.Set(10, 11, s.f22)                             // B2/H2
	jac.Set(10, 12, s.f23)                             // B2/H3
	jac.Set(10, 22, 2*(s.H1*s.F3+s.H2*s.F0-s.H3*s.F1)) // B2/F0
	jac.Set(10, 23, 2*(s.H1*s.F2-s.H2*s.F1-s.H3*s.F0)) // B2/F1
	jac.Set(10, 24, 2*(s.H1*s.F1+s.H2*s.F2+s.H3*s.F3)) // B2/F2
	jac.Set(10, 25, 2*(s.H1*s.F0-s.H2*s.F3+s.H3*s.F2)) // B2/F3
	jac.Set(10, 27, 1)                                 // B2/D2

	//m.B3 = s.f31*s.H1 + s.f32*s.H2 + s.f33*s.H3 + s.D3
	//s.f31 = 2*(-s.F0 * s.F2 + s.F3 * s.F1)
	//s.f32 = 2*(+s.F0 * s.F1 + s.F2 * s.F3)
	//s.f33 = (+s.F0 * s.F0 - s.F1 * s.F1 - s.F2 * s.F2 + s.F3 * s.F3)
	jac.Set(11, 10, s.f31)                              // B3/H1
	jac.Set(11, 11, s.f32)                              // B3/H2
	jac.Set(11, 12, s.f33)                              // B3/H3
	jac.Set(11, 22, 2*(-s.H1*s.F2+s.H2*s.F1+s.H3*s.F0)) // B3/F0
	jac.Set(11, 23, 2*(s.H1*s.F3+s.H2*s.F0-s.H3*s.F1))  // B3/F1
	jac.Set(11, 24, 2*(-s.H1*s.F0+s.H2*s.F3-s.H3*s.F2)) // B3/F2
	jac.Set(11, 25, 2*(s.H1*s.F1+s.H2*s.F2+s.H3*s.F3))  // B3/F3
	jac.Set(11, 28, 1)                                  // B3/D3

	return
}

var Kalman1JSONConfig = ""
