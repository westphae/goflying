// Package ahrs implements a Kalman filter for determining aircraft kinematic state
// based on inputs from IMU and GPS
package ahrs

import (
	"github.com/skelterjohn/go.matrix"
	"log"
	"math"
)

type KalmanState struct {
	State
}

func (s *KalmanState) CalcRollPitchHeadingUncertainty() (droll float64, dpitch float64, dheading float64) {
	droll, dpitch, dheading = VarFromQuaternion(s.E0, s.E1, s.E2, s.E3,
		math.Sqrt(s.M.Get(6, 6)), math.Sqrt(s.M.Get(7, 7)),
		math.Sqrt(s.M.Get(8, 8)), math.Sqrt(s.M.Get(9, 9)))
	return
}

// GetState returns the Kalman state of the system
func (s *KalmanState) GetState() *State {
	return &s.State
}

// GetStateMap returns the state information for analysis
func (s *KalmanState) GetStateMap() (dat *map[string]float64) {
	return
}

// Initialize the state at the start of the Kalman filter, based on current measurements
func InitializeKalman(m *Measurement) (s *KalmanState) {
	s = new(KalmanState)
	s.init(m)
	return
}

func (s *KalmanState) init(m *Measurement) {
	// Diagonal matrix of initial state uncertainties, will be squared into covariance below
	// Specifics here aren't too important--it will change very quickly
	s.M = matrix.Diagonal([]float64{
		50, 5, 5, // U*3
		0.4, 0.2, 0.5, // Z*3
		0.5, 0.5, 0.5, 0.5, // E*4
		2, 2, 2, // H*3
		65, 65, 65, // N*3
		10, 10, 2, // V*3
		0.02, 0.02, 0.02, // C*3
		0.002, 0.002, 0.002, 0.002, // F*4
		0.1, 0.1, 0.1, // D*4
		10, 10, 10, // L*4
	})
	s.M = matrix.Product(s.M, s.M)

	// Diagonal matrix of state process uncertainties per s, will be squared into covariance below
	// Tuning these is more important
	tt := math.Sqrt(60.0 * 60.0) // One-hour time constant for drift of biases V, C, F, D, L
	s.N = matrix.Diagonal([]float64{
		1, 0.1, 0.1, // U*3
		0.2, 0.1, 0.2, // Z*3
		0.02, 0.02, 0.02, 0.02, // E*4
		1, 1, 1, // H*3
		100, 100, 100, // N*3
		5 / tt, 5 / tt, 5 / tt, // V*3
		0.01 / tt, 0.01 / tt, 0.01 / tt, // C*3
		0.0001 / tt, 0.0001 / tt, 0.0001 / tt, 0.0001 / tt, // F*4
		0.1 / tt, 0.1 / tt, 0.1 / tt, // D*3
		0.1 / tt, 0.1 / tt, 0.1 / tt, // L*3
	})
	s.N = matrix.Product(s.N, s.N)

	//TODO westphae: for now just treat the case !m.UValid; if we have U, we can do a lot more!

	// Best guess at initial airspeed is initial groundspeed
	if m.WValid {
		s.U1 = math.Hypot(m.W1, m.W2)
		s.M.Set(0, 0, 14*14) // Our estimate of airspeed is better
		s.M.Set(16, 16, 10)  // Matching uncertainty of windspeed
		s.M.Set(17, 17, 10)  // Matching uncertainty of windspeed
	}

	// Best guess at initial heading is initial track
	if m.WValid && s.U1 > 5 {
		// Simplified half-angle formulae
		s.E0, s.E3 = math.Sqrt((s.U1+m.W1)/(2*s.U1)), math.Sqrt((s.U1-m.W1)/(2*s.U1))
		if m.W2 < 0 {
			s.E3 *= -1
		}
		s.M.Set(6, 6, 0.1*0.1) // Our estimate of orientation is better
		s.M.Set(7, 7, 0.1*0.1)
		s.M.Set(8, 8, 0.1*0.1)
		s.M.Set(9, 9, 0.1*0.1)
	} else { // If no groundspeed available then no idea which direction we're pointing
		s.E0 = 1 // assume east
	}

	s.F0 = 1 // Initial guess is that it's oriented pointing forward and level

	s.normalize()

	if m.MValid { //TODO westphae: could do more here to get a better Fn since we know N points north
		s.N1 = m.M1*s.e11 + m.M2*s.e12 + m.M3*s.e13
		s.N2 = m.M1*s.e21 + m.M2*s.e22 + m.M3*s.e23
		s.N3 = m.M1*s.e31 + m.M2*s.e32 + m.M3*s.e33
	} else {
		s.M.Set(13, 13, Big) // Don't try to update the magnetometer
		s.M.Set(14, 14, Big)
		s.M.Set(15, 15, Big)
		s.M.Set(29, 29, Big)
		s.M.Set(30, 30, Big)
		s.M.Set(31, 31, Big)
	}

	return
}

// Compute runs first the prediction and then the update phases of the Kalman filter
func (s *KalmanState) Compute(m *Measurement) {
	s.Predict(m.T)
	s.Update(m)
}

// Valid applies some heuristics to detect whether the computed state is valid or not
func (s *KalmanState) Valid() (ok bool) {
	ok = true

	if s.U1 < -5 {
		ok = false
	}

	if math.Abs(s.U1) > 300 || math.Abs(s.U2) > 20 || math.Abs(s.U3) > 20 ||
		math.Abs(s.V1) > 40 || math.Abs(s.V2) > 40 || math.Abs(s.V3) > 40 {
		ok = false
	}

	roll, pitch, heading := s.CalcRollPitchHeading()
	droll, dpitch, dheading := s.CalcRollPitchHeadingUncertainty()
	if droll > 2.5*Deg || dpitch > 2.5*Deg {
		log.Printf("AHRS too uncertain: roll %5.1f +/- %3.1f, pitch %4.1f +/- %3.1f, heading %5.1f +/- %3.1f\n",
			roll/Deg, droll/Deg, pitch/Deg, dpitch/Deg, heading/Deg, dheading/Deg)
		ok = false
	}

	return ok
}

// Predict performs the prediction phase of the Kalman filter
func (s *KalmanState) Predict(t float64) {
	f := s.calcJacobianState(t)
	dt := t - s.T

	s.U1 += dt * s.Z1 * G
	s.U2 += dt * s.Z2 * G
	s.U3 += dt * s.Z3 * G

	s.E0 += 0.5 * dt * (-s.H1*s.E1 - s.H2*s.E2 - s.H3*s.E3) * Deg
	s.E1 += 0.5 * dt * (+s.H1*s.E0 + s.H2*s.E3 - s.H3*s.E2) * Deg
	s.E2 += 0.5 * dt * (-s.H1*s.E3 + s.H2*s.E0 + s.H3*s.E1) * Deg
	s.E3 += 0.5 * dt * (+s.H1*s.E2 - s.H2*s.E1 + s.H3*s.E0) * Deg
	s.normalize()

	// All other state vectors are unchanged

	s.T = t

	s.M = matrix.Sum(matrix.Product(f, matrix.Product(s.M, f.Transpose())), matrix.Scaled(s.N, dt))
}

// Update applies the Kalman filter corrections given the measurements
func (s *KalmanState) Update(m *Measurement) {
	z := s.PredictMeasurement()

	//TODO westphae: for testing, if no GPS, we're probably inside at a desk - assume zero groundspeed
	if !m.WValid {
		m.W1 = 0
		m.W2 = 0
		m.W3 = 0
		m.WValid = true
	}

	y := matrix.Zeros(15, 1)
	y.Set(0, 0, m.U1-z.U1)
	y.Set(1, 0, m.U2-z.U2)
	y.Set(2, 0, m.U3-z.U3)
	y.Set(3, 0, m.W1-z.W1)
	y.Set(4, 0, m.W2-z.W2)
	y.Set(5, 0, m.W3-z.W3)
	y.Set(6, 0, m.A1-z.A1)
	y.Set(7, 0, m.A2-z.A2)
	y.Set(8, 0, m.A3-z.A3)
	y.Set(9, 0, m.B1-z.B1)
	y.Set(10, 0, m.B2-z.B2)
	y.Set(11, 0, m.B3-z.B3)
	y.Set(12, 0, m.M1-z.M1)
	y.Set(13, 0, m.M2-z.M2)
	y.Set(14, 0, m.M3-z.M3)

	h := s.calcJacobianMeasurement()

	var v float64
	// U, W, A, B, M
	if m.UValid {
		_, _, v = m.Accums[0](m.U1)
		m.M.Set(0, 0, v)
	} else {
		y.Set(0, 0, 0)
		m.M.Set(0, 0, Big)
	}
	// U2, U3 are just here to bias toward coordinated flight
	//TODO westphae: not sure I really want these to not be BIG
	m.M.Set(1, 1, 1)
	m.M.Set(2, 2, 1)

	if m.WValid {
		_, _, v = m.Accums[3](m.W1)
		m.M.Set(3, 3, v)
		_, _, v = m.Accums[4](m.W2)
		m.M.Set(4, 4, v)
		_, _, v = m.Accums[5](m.W3)
		m.M.Set(5, 5, v)
	} else {
		y.Set(3, 0, 0)
		y.Set(4, 0, 0)
		y.Set(5, 0, 0)
		m.M.Set(3, 3, Big)
		m.M.Set(4, 4, Big)
		m.M.Set(5, 5, Big)
	}

	if m.SValid {
		_, _, v = m.Accums[6](m.A1)
		m.M.Set(6, 6, v)
		_, _, v = m.Accums[7](m.A2)
		m.M.Set(7, 7, v)
		_, _, v = m.Accums[8](m.A3)
		m.M.Set(8, 8, v)
		_, _, v = m.Accums[9](m.B1)
		m.M.Set(9, 9, v)
		_, _, v = m.Accums[10](m.B2)
		m.M.Set(10, 10, v)
		_, _, v = m.Accums[11](m.B3)
		m.M.Set(11, 11, v)
	} else {
		y.Set(6, 0, 0)
		y.Set(7, 0, 0)
		y.Set(8, 0, 0)
		y.Set(9, 0, 0)
		y.Set(10, 0, 0)
		y.Set(11, 0, 0)
		m.M.Set(6, 6, Big)
		m.M.Set(7, 7, Big)
		m.M.Set(8, 8, Big)
		m.M.Set(9, 9, Big)
		m.M.Set(10, 10, Big)
		m.M.Set(11, 11, Big)
	}

	if m.MValid {
		_, _, v = m.Accums[12](m.M1)
		m.M.Set(12, 12, v)
		_, _, v = m.Accums[13](m.M2)
		m.M.Set(13, 13, v)
		_, _, v = m.Accums[14](m.M3)
		m.M.Set(14, 14, v)
	} else {
		y.Set(12, 0, 0)
		y.Set(13, 0, 0)
		y.Set(14, 0, 0)
		m.M.Set(12, 12, Big)
		m.M.Set(13, 13, Big)
		m.M.Set(14, 14, Big)
	}

	ss := matrix.Sum(matrix.Product(h, matrix.Product(s.M, h.Transpose())), m.M)

	m2, err := ss.Inverse()
	if err != nil {
		return
	}
	kk := matrix.Product(s.M, matrix.Product(h.Transpose(), m2))
	su := matrix.Product(kk, y)
	s.U1 += su.Get(0, 0)
	s.U2 += su.Get(1, 0)
	s.U3 += su.Get(2, 0)
	s.Z1 += su.Get(3, 0)
	s.Z2 += su.Get(4, 0)
	s.Z3 += su.Get(5, 0)
	s.E0 += su.Get(6, 0)
	s.E1 += su.Get(7, 0)
	s.E2 += su.Get(8, 0)
	s.E3 += su.Get(9, 0)
	s.H1 += su.Get(10, 0)
	s.H2 += su.Get(11, 0)
	s.H3 += su.Get(12, 0)
	s.N1 += su.Get(13, 0)
	s.N2 += su.Get(14, 0)
	s.N3 += su.Get(15, 0)
	s.V1 += su.Get(16, 0)
	s.V2 += su.Get(17, 0)
	s.V3 += su.Get(18, 0)
	s.C1 += su.Get(19, 0)
	s.C2 += su.Get(20, 0)
	s.C3 += su.Get(21, 0)
	s.F0 += su.Get(22, 0)
	s.F1 += su.Get(23, 0)
	s.F2 += su.Get(24, 0)
	s.F3 += su.Get(25, 0)
	s.D1 += su.Get(26, 0)
	s.D2 += su.Get(27, 0)
	s.D3 += su.Get(28, 0)
	s.L1 += su.Get(29, 0)
	s.L2 += su.Get(30, 0)
	s.L3 += su.Get(31, 0)
	s.T = m.T
	s.M = matrix.Product(matrix.Difference(matrix.Eye(32), matrix.Product(kk, h)), s.M)
	s.normalize()
}

func (s *KalmanState) PredictMeasurement() (m *Measurement) {
	m = NewMeasurement()

	m.UValid = true
	m.U1 = s.U1
	m.U2 = s.U2
	m.U3 = s.U3

	m.WValid = true
	m.W1 = s.e11*s.U1 + s.e12*s.U2 + s.e13*s.U3 + s.V1
	m.W2 = s.e21*s.U1 + s.e22*s.U2 + s.e23*s.U3 + s.V2
	m.W3 = s.e31*s.U1 + s.e32*s.U2 + s.e33*s.U3 + s.V3

	m.SValid = true
	// Include pseudoforces from non-inertial frame!  Why we see "contamination" of accel from gyro
	h1 := s.H1*s.e11 + s.H2*s.e21 + s.H3*s.e31
	h2 := s.H1*s.e12 + s.H2*s.e22 + s.H3*s.e32
	h3 := s.H1*s.e13 + s.H2*s.e23 + s.H3*s.e33
	a1 := -s.Z1 + (h3*s.U2-h2*s.U3)*Deg/G - s.e31
	a2 := -s.Z2 + (h1*s.U3-h3*s.U1)*Deg/G - s.e32
	a3 := -s.Z3 + (h2*s.U1-h1*s.U2)*Deg/G - s.e33

	m.A1 = s.f11*a1 + s.f12*a2 + s.f13*a3 + s.C1
	m.A2 = s.f21*a1 + s.f22*a2 + s.f23*a3 + s.C2
	m.A3 = s.f31*a1 + s.f32*a2 + s.f33*a3 + s.C3

	m.B1 = s.f11*h1 + s.f12*h2 + s.f13*h3 + s.D1
	m.B2 = s.f21*h1 + s.f22*h2 + s.f23*h3 + s.D2
	m.B3 = s.f31*h1 + s.f32*h2 + s.f33*h3 + s.D3

	m.MValid = true
	m1 := s.N1*s.e11 + s.N2*s.e21 + s.N3*s.e31 + s.L1
	m2 := s.N1*s.e12 + s.N2*s.e22 + s.N3*s.e32 + s.L2
	m3 := s.N1*s.e13 + s.N2*s.e23 + s.N3*s.e33 + s.L3
	m.M1 = s.f11*m1 + s.f12*m2 + s.f13*m3
	m.M2 = s.f21*m1 + s.f22*m2 + s.f23*m3
	m.M3 = s.f31*m1 + s.f32*m2 + s.f33*m3

	m.T = s.T

	return
}

func (s *KalmanState) calcJacobianState(t float64) (jac *matrix.DenseMatrix) {
	dt := t - s.T

	jac = matrix.Eye(32)
	// U*3, Z*3, E*4, H*3, N*3,
	// V*3, C*3, F*4, D*3, L*3

	//s.U1 += dt*s.Z1*G
	jac.Set(0, 3, dt*G) // U1/Z1
	//s.U2 += dt*s.Z2*G
	jac.Set(1, 4, dt*G) // U2/Z2
	//s.U3 += dt*s.Z3*G
	jac.Set(2, 5, dt*G) // U3/Z3

	//s.E0 += 0.5*dt*(-s.H1*s.E1 - s.H2*s.E2 - s.H3*s.E3)*Deg
	jac.Set(6, 7, -0.5*dt*s.H1*Deg)  // E0/E1
	jac.Set(6, 8, -0.5*dt*s.H2*Deg)  // E0/E2
	jac.Set(6, 9, -0.5*dt*s.H3*Deg)  // E0/E3
	jac.Set(6, 10, -0.5*dt*s.E1*Deg) // E0/H1
	jac.Set(6, 11, -0.5*dt*s.E2*Deg) // E0/H2
	jac.Set(6, 12, -0.5*dt*s.E3*Deg) // E0/H3

	//s.E1 += 0.5*dt*(+s.H1*s.E0 + s.H2*s.E3 - s.H3*s.E2)*Deg
	jac.Set(7, 6, +0.5*dt*s.H1*Deg)  // E1/E0
	jac.Set(7, 8, -0.5*dt*s.H3*Deg)  // E1/E2
	jac.Set(7, 9, +0.5*dt*s.H2*Deg)  // E1/E3
	jac.Set(7, 10, +0.5*dt*s.E0*Deg) // E1/H1
	jac.Set(7, 11, +0.5*dt*s.E3*Deg) // E1/H2
	jac.Set(7, 12, -0.5*dt*s.E2*Deg) // E1/H3

	//s.E2 += 0.5*dt*(-s.H1*s.E3 + s.H2*s.E0 + s.H3*s.E1)*Deg
	jac.Set(8, 6, +0.5*dt*s.H2*Deg)  // E2/E0
	jac.Set(8, 7, +0.5*dt*s.H3*Deg)  // E2/E1
	jac.Set(8, 9, -0.5*dt*s.H1*Deg)  // E2/E3
	jac.Set(8, 10, -0.5*dt*s.E3*Deg) // E2/H1
	jac.Set(8, 11, +0.5*dt*s.E0*Deg) // E2/H2
	jac.Set(8, 12, +0.5*dt*s.E1*Deg) // E2/H3

	//s.E3 += 0.5*dt*(+s.H1*s.E2 - s.H2*s.E1 + s.H3*s.E0)*Deg
	jac.Set(9, 6, +0.5*dt*s.H3*Deg)  // E3/E0
	jac.Set(9, 7, -0.5*dt*s.H2*Deg)  // E3/E1
	jac.Set(9, 8, +0.5*dt*s.H1*Deg)  // E3/E2
	jac.Set(9, 10, +0.5*dt*s.E2*Deg) // E3/H1
	jac.Set(9, 11, -0.5*dt*s.E1*Deg) // E3/H2
	jac.Set(9, 12, +0.5*dt*s.E0*Deg) // E3/H3

	return
}

func (s *KalmanState) calcJacobianMeasurement() (jac *matrix.DenseMatrix) {

	jac = matrix.Zeros(15, 32)
	// U*3, Z*3, E*4, H*3, N*3,
	// V*3, C*3, F*4, D*3, L*3
	// U*3, W*3, A*3, B*3, M*3

	//m.U1 = s.U1
	jac.Set(0, 0, 1) // U1/U1
	//m.U2 = s.U2
	jac.Set(1, 1, 1) // U2/U2
	//m.U3 = s.U3
	jac.Set(2, 2, 1) // U3/U3

	w1 := s.e11*s.U1 + s.e12*s.U2 + s.e13*s.U3
	//s.e11 = 2*(+s.E0 * s.E0 + s.E1 * s.E1 - 0.5)
	//s.e12 = 2*(-s.E0 * s.E3 + s.E1 * s.E2)
	//s.e13 = 2*(+s.E0 * s.E2 + s.E1 * s.E3)
	jac.Set(3, 0, s.e11) // W1/U1
	jac.Set(3, 1, s.e12) // W1/U2
	jac.Set(3, 2, s.e13) // W1/U3
	jac.Set(3, 6,        // W1/E0
		2*(+s.E0*s.U1-s.E3*s.U2+s.E2*s.U3)-
			2*w1*s.E0)
	jac.Set(3, 7, // W1/E1
		2*(+s.E1*s.U1+s.E2*s.U2+s.E3*s.U3)-
			2*w1*s.E1)
	jac.Set(3, 8, // W1/E2
		2*(-s.E2*s.U1+s.E1*s.U2+s.E0*s.U3)-
			2*w1*s.E2)
	jac.Set(3, 9, // W1/E3
		2*(-s.E3*s.U1-s.E0*s.U2+s.E1*s.U3)-
			2*w1*s.E3)
	jac.Set(3, 16, 1) // W1/V1

	w2 := s.e21*s.U1 + s.e22*s.U2 + s.e23*s.U3
	//s.e21 = 2*(+s.E0 * s.E3 + s.E2 * s.E1)
	//s.e22 = 2*(+s.E0 * s.E0 + s.E2 * s.E2 - 0.5)
	//s.e23 = 2*(-s.E0 * s.E1 + s.E2 * s.E3)
	jac.Set(4, 0, s.e21) // W2/U1
	jac.Set(4, 1, s.e22) // W2/U2
	jac.Set(4, 2, s.e23) // W2/U3
	jac.Set(4, 6,        // W2/E0
		2*(+s.E3*s.U1+s.E0*s.U2-s.E1*s.U3)-
			2*w2*s.E0)
	jac.Set(4, 7, // W2/E1
		2*(+s.E2*s.U1-s.E1*s.U2-s.E0*s.U3)-
			2*w2*s.E1)
	jac.Set(4, 8, // W2/E2
		2*(+s.E1*s.U1+s.E2*s.U2+s.E3*s.U3)-
			2*w2*s.E2)
	jac.Set(4, 9, // W2/E3
		2*(+s.E0*s.U1-s.E3*s.U2+s.E2*s.U3)-
			2*w2*s.E3)
	jac.Set(4, 17, 1) // W2/V2

	w3 := s.e31*s.U1 + s.e32*s.U2 + s.e33*s.U3
	//s.e31 = 2*(-s.E0 * s.E2 + s.E3 * s.E1)
	//s.e32 = 2*(+s.E0 * s.E1 + s.E3 * s.E2)
	//s.e33 = 2*(+s.E0 * s.E0 + s.E3 * s.E3 - 0.5)
	jac.Set(5, 0, s.e31) // W3/U1
	jac.Set(5, 1, s.e32) // W3/U2
	jac.Set(5, 2, s.e33) // W3/U3
	jac.Set(5, 6,        // W3/E0
		2*(-s.E2*s.U1+s.E1*s.U2+s.E0*s.U3)-
			2*w3*s.E0)
	jac.Set(5, 7, // W3/E1
		2*(+s.E3*s.U1+s.E0*s.U2-s.E1*s.U3)-
			2*w3*s.E1)
	jac.Set(5, 8, // W3/E2
		2*(-s.E0*s.U1+s.E3*s.U2-s.E2*s.U3)-
			2*w3*s.E2)
	jac.Set(5, 9, // W3/E3
		2*(+s.E1*s.U1+s.E2*s.U2+s.E3*s.U3)-
			2*w3*s.E3)
	jac.Set(5, 18, 1) // W3/V3

	h1 := s.H1*s.e11 + s.H2*s.e21 + s.H3*s.e31
	h2 := s.H1*s.e12 + s.H2*s.e22 + s.H3*s.e32
	h3 := s.H1*s.e13 + s.H2*s.e23 + s.H3*s.e33
	a1 := -s.Z1 + (h3*s.U2-h2*s.U3)*Deg/G - s.e31
	a2 := -s.Z2 + (h1*s.U3-h3*s.U1)*Deg/G - s.e32
	a3 := -s.Z3 + (h2*s.U1-h1*s.U2)*Deg/G - s.e33

	ae1 := s.f11*(a1+s.Z1) + s.f12*(a2+s.Z2) + s.f13*(a3+s.Z3)
	af1 := s.f11*a1 + s.f12*a2 + s.f13*a3
	jac.Set(6, 0, (s.f13*h2-s.f12*h3)*Deg/G) // A1/U1
	jac.Set(6, 1, (s.f11*h3-s.f13*h1)*Deg/G) // A1/U2
	jac.Set(6, 2, (s.f12*h1-s.f11*h2)*Deg/G) // A1/U3
	jac.Set(6, 3, -s.f11)                    // A1/Z1
	jac.Set(6, 4, -s.f12)                    // A1/Z2
	jac.Set(6, 5, -s.f13)                    // A1/Z3
	jac.Set(6, 6, 2*Deg/G*(                  // A1/E0
	s.f11*(s.H1*(s.E2*s.U2+s.E3*s.U3)+s.H2*(-s.E1*s.U2-s.E0*s.U3)+s.H3*(s.E0*s.U2-s.E1*s.U3))+
		s.f12*(s.H1*(s.E0*s.U3-s.E2*s.U1)+s.H2*(s.E3*s.U3+s.E1*s.U1)+s.H3*(-s.E2*s.U3-s.E0*s.U1))+
		s.f13*(s.H1*(-s.E3*s.U1-s.E0*s.U2)+s.H2*(s.E0*s.U1-s.E3*s.U2)+s.H3*(s.E1*s.U1+s.E2*s.U2)))-
		2*ae1*s.E0-
		2*(s.f11*(-s.E2)+s.f12*(s.E1)+s.f13*(s.E0)))
	jac.Set(6, 7, 2*Deg/G*( // A1/E1
	s.f11*(s.H1*(s.E3*s.U2-s.E2*s.U3)+s.H2*(-s.E0*s.U2+s.E1*s.U3)+s.H3*(-s.E1*s.U2-s.E0*s.U3))+
		s.f12*(s.H1*(s.E1*s.U3-s.E3*s.U1)+s.H2*(s.E2*s.U3+s.E0*s.U1)+s.H3*(s.E3*s.U3+s.E1*s.U1))+
		s.f13*(s.H1*(s.E2*s.U1-s.E1*s.U2)+s.H2*(-s.E1*s.U1-s.E2*s.U2)+s.H3*(s.E0*s.U1-s.E3*s.U2)))-
		2*ae1*s.E1-
		2*(s.f11*(s.E3)+s.f12*(s.E0)+s.f13*(-s.E1)))
	jac.Set(6, 8, 2*Deg/G*( // A1/E2
	s.f11*(s.H1*(s.E0*s.U2-s.E1*s.U3)+s.H2*(s.E3*s.U2-s.E2*s.U3)+s.H3*(-s.E2*s.U2-s.E3*s.U3))+
		s.f12*(s.H1*(-s.E2*s.U3-s.E0*s.U1)+s.H2*(s.E1*s.U3-s.E3*s.U1)+s.H3*(-s.E0*s.U3+s.E2*s.U1))+
		s.f13*(s.H1*(s.E1*s.U1+s.E2*s.U2)+s.H2*(s.E2*s.U1-s.E1*s.U2)+s.H3*(s.E3*s.U1+s.E0*s.U2)))-
		2*ae1*s.E2-
		2*(s.f11*(-s.E0)+s.f12*(s.E3)+s.f13*(-s.E2)))
	jac.Set(6, 9, 2*Deg/G*( // A1/E3
	s.f11*(s.H1*(s.E1*s.U2+s.E0*s.U3)+s.H2*(s.E2*s.U2+s.E3*s.U3)+s.H3*(s.E3*s.U2-s.E2*s.U3))+
		s.f12*(s.H1*(-s.E3*s.U3-s.E1*s.U1)+s.H2*(s.E0*s.U3-s.E2*s.U1)+s.H3*(s.E1*s.U3-s.E3*s.U1))+
		s.f13*(s.H1*(-s.E0*s.U1+s.E3*s.U2)+s.H2*(-s.E3*s.U1-s.E0*s.U2)+s.H3*(s.E2*s.U1-s.E1*s.U2)))-
		2*ae1*s.E3-
		2*(s.f11*(s.E1)+s.f12*(s.E2)+s.f13*(s.E3)))
	jac.Set(6, 10, Deg/G*( // A1/H1
	s.f11*(s.U2*s.e13-s.U3*s.e12)+
		s.f12*(s.U3*s.e11-s.U1*s.e13)+
		s.f13*(s.U1*s.e12-s.U2*s.e11)))
	jac.Set(6, 11, Deg/G*( // A1/H2
	s.f11*(s.U2*s.e23-s.U3*s.e22)+
		s.f12*(s.U3*s.e21-s.U1*s.e23)+
		s.f13*(s.U1*s.e22-s.U2*s.e21)))
	jac.Set(6, 12, Deg/G*( // A1/H3
	s.f11*(s.U2*s.e33-s.U3*s.e32)+
		s.f12*(s.U3*s.e31-s.U1*s.e33)+
		s.f13*(s.U1*s.e32-s.U2*s.e31)))
	jac.Set(6, 19, 1) // A1/C1
	jac.Set(6, 22,    // A1/F0
		2*(+s.F0*a1-s.F3*a2+s.F2*a3)-
			2*af1*s.F0)
	jac.Set(6, 23, // A1/F1
		2*(+s.F1*a1+s.F2*a2+s.F3*a3)-
			2*af1*s.F1)
	jac.Set(6, 24, // A1/F2
		2*(-s.F2*a1+s.F1*a2+s.F0*a3)-
			2*af1*s.F2)
	jac.Set(6, 25, // A1/F3
		2*(-s.F3*a1-s.F0*a2+s.F1*a3)-
			2*af1*s.F3)

	aa2 := s.f21*(a1+s.Z1) + s.f22*(a2+s.Z2) + s.f23*(a3+s.Z3)
	af2 := s.f21*a1 + s.f22*a2 + s.f23*a3
	jac.Set(7, 0, (h2*s.f23-h3*s.f22)*Deg/G) // A2/U1
	jac.Set(7, 1, (h3*s.f21-h1*s.f23)*Deg/G) // A2/U2
	jac.Set(7, 2, (h1*s.f22-h2*s.f21)*Deg/G) // A2/U3
	jac.Set(7, 3, -s.f21)                    // A2/Z1
	jac.Set(7, 4, -s.f22)                    // A2/Z2
	jac.Set(7, 5, -s.f23)                    // A2/Z3
	jac.Set(7, 6, 2*Deg/G*(                  // A2/E0
	s.f21*(s.H1*(s.E2*s.U2+s.E3*s.U3)+s.H2*(-s.E1*s.U2-s.E0*s.U3)+s.H3*(s.E0*s.U2-s.E1*s.U3))+
		s.f22*(s.H1*(s.E0*s.U3-s.E2*s.U1)+s.H2*(s.E3*s.U3+s.E1*s.U1)+s.H3*(-s.E2*s.U3-s.E0*s.U1))+
		s.f23*(s.H1*(-s.E3*s.U1-s.E0*s.U2)+s.H2*(s.E0*s.U1-s.E3*s.U2)+s.H3*(s.E1*s.U1+s.E2*s.U2)))-
		2*aa2*s.E0-
		2*(s.f21*(-s.E2)+s.f22*(s.E1)+s.f23*(s.E0)))
	jac.Set(7, 7, 2*Deg/G*( // A2/E1
	s.f21*(s.H1*(s.E3*s.U2-s.E2*s.U3)+s.H2*(-s.E0*s.U2+s.E1*s.U3)+s.H3*(-s.E1*s.U2-s.E0*s.U3))+
		s.f22*(s.H1*(s.E1*s.U3-s.E3*s.U1)+s.H2*(s.E2*s.U3+s.E0*s.U1)+s.H3*(s.E3*s.U3+s.E1*s.U1))+
		s.f23*(s.H1*(s.E2*s.U1-s.E1*s.U2)+s.H2*(-s.E1*s.U1-s.E2*s.U2)+s.H3*(s.E0*s.U1-s.E3*s.U2)))-
		2*aa2*s.E1-
		2*(s.f21*(s.E3)+s.f22*(s.E0)+s.f23*(-s.E1)))
	jac.Set(7, 8, 2*Deg/G*( // A2/E2
	s.f21*(s.H1*(s.E0*s.U2-s.E1*s.U3)+s.H2*(s.E3*s.U2-s.E2*s.U3)+s.H3*(-s.E2*s.U2-s.E3*s.U3))+
		s.f22*(s.H1*(-s.E2*s.U3-s.E0*s.U1)+s.H2*(s.E1*s.U3-s.E3*s.U1)+s.H3*(-s.E0*s.U3+s.E2*s.U1))+
		s.f23*(s.H1*(s.E1*s.U1+s.E2*s.U2)+s.H2*(s.E2*s.U1-s.E1*s.U2)+s.H3*(s.E3*s.U1+s.E0*s.U2)))-
		2*aa2*s.E2-
		2*(s.f21*(-s.E0)+s.f22*(s.E3)+s.f23*(-s.E2)))
	jac.Set(7, 9, 2*Deg/G*( // A2/E3
	s.f21*(s.H1*(s.E1*s.U2+s.E0*s.U3)+s.H2*(s.E2*s.U2+s.E3*s.U3)+s.H3*(s.E3*s.U2-s.E2*s.U3))+
		s.f22*(s.H1*(-s.E3*s.U3-s.E1*s.U1)+s.H2*(s.E0*s.U3-s.E2*s.U1)+s.H3*(s.E1*s.U3-s.E3*s.U1))+
		s.f23*(s.H1*(-s.E0*s.U1+s.E3*s.U2)+s.H2*(-s.E3*s.U1-s.E0*s.U2)+s.H3*(s.E2*s.U1-s.E1*s.U2)))-
		2*aa2*s.E3-
		2*(s.f21*(s.E1)+s.f22*(s.E2)+s.f23*(s.E3)))
	jac.Set(7, 10, Deg/G*( // A2/H1
	s.f21*(s.U2*s.e13-s.U3*s.e12)+
		s.f22*(s.U3*s.e11-s.U1*s.e13)+
		s.f23*(s.U1*s.e12-s.U2*s.e11)))
	jac.Set(7, 11, Deg/G*( // A2/H2
	s.f21*(s.U2*s.e23-s.U3*s.e22)+
		s.f22*(s.U3*s.e21-s.U1*s.e23)+
		s.f23*(s.U1*s.e22-s.U2*s.e21)))
	jac.Set(7, 12, Deg/G*( // A2/H3
	s.f21*(s.U2*s.e33-s.U3*s.e32)+
		s.f22*(s.U3*s.e31-s.U1*s.e33)+
		s.f23*(s.U1*s.e32-s.U2*s.e31)))
	jac.Set(7, 20, 1) // A2/C2
	jac.Set(7, 22,    // A2/F0
		2*(+s.F3*a1+s.F0*a2-s.F1*a3)-
			2*af2*s.F0)
	jac.Set(7, 23, // A2/F1
		2*(+s.F2*a1-s.F1*a2-s.F0*a3)-
			2*af2*s.F1)
	jac.Set(7, 24, // A2/F2
		2*(+s.F1*a1+s.F2*a2+s.F3*a3)-
			2*af2*s.F2)
	jac.Set(7, 25, // A2/F3
		2*(+s.F0*a1-s.F3*a2+s.F2*a3)-
			2*af2*s.F3)

	aa3 := s.f31*(a1+s.Z1) + s.f32*(a2+s.Z2) + s.f33*(a3+s.Z3)
	af3 := s.f31*a1 + s.f32*a2 + s.f33*a3
	jac.Set(8, 0, (h2*s.f33-h3*s.f32)*Deg/G) // A3/U1
	jac.Set(8, 1, (h3*s.f31-h1*s.f33)*Deg/G) // A3/U2
	jac.Set(8, 2, (h1*s.f32-h2*s.f31)*Deg/G) // A3/U3
	jac.Set(8, 3, -s.f31)                    // A3/Z1
	jac.Set(8, 4, -s.f32)                    // A3/Z2
	jac.Set(8, 5, -s.f33)                    // A3/Z3
	jac.Set(8, 6, 2*Deg/G*(                  // A3/E0
	s.f31*(s.H1*(s.E2*s.U2+s.E3*s.U3)+s.H2*(-s.E1*s.U2-s.E0*s.U3)+s.H3*(s.E0*s.U2-s.E1*s.U3))+
		s.f32*(s.H1*(s.E0*s.U3-s.E2*s.U1)+s.H2*(s.E3*s.U3+s.E1*s.U1)+s.H3*(-s.E2*s.U3-s.E0*s.U1))+
		s.f33*(s.H1*(-s.E3*s.U1-s.E0*s.U2)+s.H2*(s.E0*s.U1-s.E3*s.U2)+s.H3*(s.E1*s.U1+s.E2*s.U2)))-
		2*aa3*s.E0-
		2*(s.f31*(-s.E2)+s.f32*(s.E1)+s.f33*(s.E0)))
	jac.Set(8, 7, 2*Deg/G*( // A3/E1
	s.f31*(s.H1*(s.E3*s.U2-s.E2*s.U3)+s.H2*(-s.E0*s.U2+s.E1*s.U3)+s.H3*(-s.E1*s.U2-s.E0*s.U3))+
		s.f32*(s.H1*(s.E1*s.U3-s.E3*s.U1)+s.H2*(s.E2*s.U3+s.E0*s.U1)+s.H3*(s.E3*s.U3+s.E1*s.U1))+
		s.f33*(s.H1*(s.E2*s.U1-s.E1*s.U2)+s.H2*(-s.E1*s.U1-s.E2*s.U2)+s.H3*(s.E0*s.U1-s.E3*s.U2)))-
		2*aa3*s.E1-
		2*(s.f31*(s.E3)+s.f32*(s.E0)+s.f33*(-s.E1)))
	jac.Set(8, 8, 2*Deg/G*( // A3/E2
	s.f31*(s.H1*(s.E0*s.U2-s.E1*s.U3)+s.H2*(s.E3*s.U2-s.E2*s.U3)+s.H3*(-s.E2*s.U2-s.E3*s.U3))+
		s.f32*(s.H1*(-s.E2*s.U3-s.E0*s.U1)+s.H2*(s.E1*s.U3-s.E3*s.U1)+s.H3*(-s.E0*s.U3+s.E2*s.U1))+
		s.f33*(s.H1*(s.E1*s.U1+s.E2*s.U2)+s.H2*(s.E2*s.U1-s.E1*s.U2)+s.H3*(s.E3*s.U1+s.E0*s.U2)))-
		2*aa3*s.E2-
		2*(s.f31*(-s.E0)+s.f32*(s.E3)+s.f33*(-s.E2)))
	jac.Set(8, 9, 2*Deg/G*( // A3/E3
	s.f31*(s.H1*(s.E1*s.U2+s.E0*s.U3)+s.H2*(s.E2*s.U2+s.E3*s.U3)+s.H3*(s.E3*s.U2-s.E2*s.U3))+
		s.f32*(s.H1*(-s.E3*s.U3-s.E1*s.U1)+s.H2*(s.E0*s.U3-s.E2*s.U1)+s.H3*(s.E1*s.U3-s.E3*s.U1))+
		s.f33*(s.H1*(-s.E0*s.U1+s.E3*s.U2)+s.H2*(-s.E3*s.U1-s.E0*s.U2)+s.H3*(s.E2*s.U1-s.E1*s.U2)))-
		2*aa3*s.E3-
		2*(s.f31*(s.E1)+s.f32*(s.E2)+s.f33*(s.E3)))
	jac.Set(8, 10, Deg/G*( // A3/H1
	s.f31*(s.U2*s.e13-s.U3*s.e12)+
		s.f32*(s.U3*s.e11-s.U1*s.e13)+
		s.f33*(s.U1*s.e12-s.U2*s.e11)))
	jac.Set(8, 11, Deg/G*( // A3/H2
	s.f31*(s.U2*s.e23-s.U3*s.e22)+
		s.f32*(s.U3*s.e21-s.U1*s.e23)+
		s.f33*(s.U1*s.e22-s.U2*s.e21)))
	jac.Set(8, 12, Deg/G*( // A3/H3
	s.f31*(s.U2*s.e33-s.U3*s.e32)+
		s.f32*(s.U3*s.e31-s.U1*s.e33)+
		s.f33*(s.U1*s.e32-s.U2*s.e31)))
	jac.Set(8, 21, 1) // A3/C3
	jac.Set(8, 22,    // A3/F0
		2*(-s.F2*a1+s.F1*a2+s.F0*a3)-
			2*af3*s.F0)
	jac.Set(8, 23, // A3/F1
		2*(+s.F3*a1+s.F0*a2-s.F1*a3)-
			2*af3*s.F1)
	jac.Set(8, 24, // A3/F2
		2*(-s.F0*a1+s.F3*a2-s.F2*a3)-
			2*af3*s.F2)
	jac.Set(8, 25, // A3/F3
		2*(+s.F1*a1+s.F2*a2+s.F3*a3)-
			2*af3*s.F3)

	b1 := s.f11*h1 + s.f12*h2 + s.f13*h3
	bf1 := b1 + s.D1
	jac.Set(9, 6, // B1/E0
		2*(s.E0*s.H1+s.E3*s.H2-s.E2*s.H3)*s.f11+
			2*(-s.E3*s.H1+s.E0*s.H2+s.E1*s.H3)*s.f12+
			2*(s.E2*s.H1-s.E1*s.H2+s.E0*s.H3)*s.f13-
			2*b1*s.E0)
	jac.Set(9, 7, // B1/E1
		2*(s.E1*s.H1+s.E2*s.H2+s.E3*s.H3)*s.f11+
			2*(s.E2*s.H1-s.E1*s.H2+s.E0*s.H3)*s.f12+
			2*(s.E3*s.H1-s.E0*s.H2-s.E1*s.H3)*s.f13-
			2*b1*s.E1)
	jac.Set(9, 8, // B1/E2
		2*(-s.E2*s.H1+s.E1*s.H2-s.E0*s.H3)*s.f11+
			2*(s.E1*s.H1+s.E2*s.H2+s.E3*s.H3)*s.f12+
			2*(s.E0*s.H1+s.E3*s.H2-s.E2*s.H3)*s.f13-
			2*b1*s.E2)
	jac.Set(9, 9, // B1/E3
		2*(-s.E3*s.H1+s.E0*s.H2+s.E1*s.H3)*s.f11+
			2*(-s.E0*s.H1-s.E3*s.H2+s.E2*s.H3)*s.f12+
			2*(s.E1*s.H1+s.E2*s.H2+s.E3*s.H3)*s.f13-
			2*b1*s.E3)
	jac.Set(9, 10, s.e11*s.f11+s.e12*s.f12+s.e13*s.f13) // B1/H1
	jac.Set(9, 11, s.e21*s.f11+s.e22*s.f12+s.e23*s.f13) // B1/H2
	jac.Set(9, 12, s.e31*s.f11+s.e32*s.f12+s.e33*s.f13) // B1/H3
	jac.Set(9, 22, 2*(h1*s.F0-h2*s.F3+h3*s.F2)-         // B1/F0
		2*bf1*s.F0)
	jac.Set(9, 23, 2*(h1*s.F1+h2*s.F2+h3*s.F3)- // B1/F1
		2*bf1*s.F1)
	jac.Set(9, 24, 2*(-h1*s.F2+h2*s.F1+h3*s.F0)- // B1/F2
		2*bf1*s.F2)
	jac.Set(9, 25, 2*(-h1*s.F3-h2*s.F0+h3*s.F1)- // B1/F3
		2*bf1*s.F3)
	jac.Set(9, 26, 1) // B1/D1

	b2 := s.f21*h1 + s.f22*h2 + s.f23*h3
	bf2 := b2 + s.D2
	jac.Set(10, 6, // B2/E0
		2*(s.E0*s.H1+s.E3*s.H2-s.E2*s.H3)*s.f21+
			2*(-s.E3*s.H1+s.E0*s.H2+s.E1*s.H3)*s.f22+
			2*(s.E2*s.H1-s.E1*s.H2+s.E0*s.H3)*s.f23-
			2*b2*s.E0)
	jac.Set(10, 7, // B2/E1
		2*(s.E1*s.H1+s.E2*s.H2+s.E3*s.H3)*s.f21+
			2*(s.E2*s.H1-s.E1*s.H2+s.E0*s.H3)*s.f22+
			2*(s.E3*s.H1-s.E0*s.H2-s.E1*s.H3)*s.f23-
			2*b2*s.E1)
	jac.Set(10, 8, // B2/E2
		2*(-s.E2*s.H1+s.E1*s.H2-s.E0*s.H3)*s.f21+
			2*(s.E1*s.H1+s.E2*s.H2+s.E3*s.H3)*s.f22+
			2*(s.E0*s.H1+s.E3*s.H2-s.E2*s.H3)*s.f23-
			2*b2*s.E2)
	jac.Set(10, 9, // B2/E3
		2*(-s.E3*s.H1+s.E0*s.H2+s.E1*s.H3)*s.f21+
			2*(-s.E0*s.H1-s.E3*s.H2+s.E2*s.H3)*s.f22+
			2*(s.E1*s.H1+s.E2*s.H2+s.E3*s.H3)*s.f23-
			2*b2*s.E3)
	jac.Set(10, 10, s.e11*s.f21+s.e12*s.f22+s.e13*s.f23) // B2/H1
	jac.Set(10, 11, s.e21*s.f21+s.e22*s.f22+s.e23*s.f23) // B2/H2
	jac.Set(10, 12, s.e31*s.f21+s.e32*s.f22+s.e33*s.f23) // B2/H3
	jac.Set(10, 22, 2*(h1*s.F3+h2*s.F0-h3*s.F1)-         // B2/F0
		2*bf2*s.F0)
	jac.Set(10, 23, 2*(h1*s.F2-h2*s.F1-h3*s.F0)- // B2/F1
		2*bf2*s.F1)
	jac.Set(10, 24, 2*(h1*s.F1+h2*s.F2+h3*s.F3)- // B2/F2
		2*bf2*s.F2)
	jac.Set(10, 25, 2*(h1*s.F0-h2*s.F3+h3*s.F2)- // B2/F3
		2*bf2*s.F3)
	jac.Set(10, 27, 1) // B2/D2

	b3 := s.f31*h1 + s.f32*h2 + s.f33*h3
	bf3 := b3 + s.D3
	jac.Set(11, 6, // B3/E0
		2*(s.E0*s.H1+s.E3*s.H2-s.E2*s.H3)*s.f31+
			2*(-s.E3*s.H1+s.E0*s.H2+s.E1*s.H3)*s.f32+
			2*(s.E2*s.H1-s.E1*s.H2+s.E0*s.H3)*s.f33-
			2*b3*s.E0)
	jac.Set(11, 7, // B3/E1
		2*(s.E1*s.H1+s.E2*s.H2+s.E3*s.H3)*s.f31+
			2*(s.E2*s.H1-s.E1*s.H2+s.E0*s.H3)*s.f32+
			2*(s.E3*s.H1-s.E0*s.H2-s.E1*s.H3)*s.f33-
			2*b3*s.E1)
	jac.Set(11, 8, // B3/E2
		2*(-s.E2*s.H1+s.E1*s.H2-s.E0*s.H3)*s.f31+
			2*(s.E1*s.H1+s.E2*s.H2+s.E3*s.H3)*s.f32+
			2*(s.E0*s.H1+s.E3*s.H2-s.E2*s.H3)*s.f33-
			2*b3*s.E2)
	jac.Set(11, 9, // B3/E2
		2*(-s.E3*s.H1+s.E0*s.H2+s.E1*s.H3)*s.f31+
			2*(-s.E0*s.H1-s.E3*s.H2+s.E2*s.H3)*s.f32+
			2*(s.E1*s.H1+s.E2*s.H2+s.E3*s.H3)*s.f33-
			2*b3*s.E3)
	jac.Set(11, 10, s.e11*s.f31+s.e12*s.f32+s.e13*s.f33) // B3/H1
	jac.Set(11, 11, s.e21*s.f31+s.e22*s.f32+s.e23*s.f33) // B3/H2
	jac.Set(11, 12, s.e31*s.f31+s.e32*s.f32+s.e33*s.f33) // B3/H3
	jac.Set(11, 22, 2*(-h1*s.F2+h2*s.F1+h3*s.F0)-        // B3/F0
		2*bf3*s.F0)
	jac.Set(11, 23, 2*(h1*s.F3+h2*s.F0-h3*s.F1)- // B3/F1
		2*bf3*s.F1)
	jac.Set(11, 24, 2*(-h1*s.F0+h2*s.F3-h3*s.F2)- // B3/F2
		2*bf3*s.F2)
	jac.Set(11, 25, 2*(h1*s.F1+h2*s.F2+h3*s.F3)- // B3/F3
		2*bf3*s.F3)
	jac.Set(11, 28, 1) // B3/D3

	//TODO westphae: fix these
	/*
		m1 :=  s.N1*s.e11 + s.N2*s.e21 + s.N3*s.e31 + s.L1
		m2 :=  s.N1*s.e12 + s.N2*s.e22 + s.N3*s.e32 + s.L2
		m3 :=  s.N1*s.e13 + s.N2*s.e23 + s.N3*s.e33 + s.L3

		m.M1 = s.f11*m1 + s.f21*m2 + s.f31*m3 + s.L1
		jac.Set(12, 6, 2*(+s.E0*s.N1 - s.E3*s.N2 + s.E2*s.N3))         // M1/E0
		jac.Set(12, 7, 2*(+s.E1*s.N1 + s.E2*s.N2 + s.E3*s.N3))         // M1/E1
		jac.Set(12, 8, 2*(-s.E2*s.N1 + s.E1*s.N2 + s.E0*s.N3))         // M1/E2
		jac.Set(12, 9, 2*(-s.E3*s.N1 - s.E0*s.N2 + s.E1*s.N3))         // M1/E3
		jac.Set(12, 13, 2*(s.E1*s.E1+s.E0*s.E0-0.5))                   // M1/N1
		jac.Set(12, 14, 2*(s.E1*s.E2-s.E0*s.E3))                       // M1/N2
		jac.Set(12, 15, 2*(s.E1*s.E3+s.E0*s.E2))                       // M1/N3
		jac.Set(12, 29, 1)                                             // M1/L1

		m.M2 = s.f12*m1 + s.f22*m2 + s.f32*m3 + s.L2
		jac.Set(13, 6,  2*(+s.E3*s.N1 + s.E0*s.N2 - s.E1*s.N3))        // M2/E0
		jac.Set(13, 7,  2*(+s.E2*s.N1 - s.E1*s.N2 - s.E0*s.N3))        // M2/E1
		jac.Set(13, 8,  2*(+s.E1*s.N1 + s.E2*s.N2 + s.E3*s.N3))        // M2/E2
		jac.Set(13, 9,  2*(+s.E0*s.N1 - s.E3*s.N2 + s.E2*s.N3))        // M2/E3
		jac.Set(13, 13, 2*(s.E2*s.E1 + s.E0*s.E3))                     // M2/N1
		jac.Set(13, 14, 2*(s.E2*s.E2 + s.E0*s.E0 - 0.5))               // M2/N2
		jac.Set(13, 15, 2*(s.E2*s.E3 - s.E0*s.E1))                     // M2/N3
		jac.Set(13, 30, 1)                                             // M2/L2

		m.M3 = s.f13*m1 + s.f23*m2 + s.f33*m3 + s.L3
		jac.Set(14, 6,  2*(-s.E2*s.N1 + s.E1*s.N2 + s.E0*s.N3))        // M3/E0
		jac.Set(14, 7,  2*(+s.E3*s.N1 + s.E0*s.N2 - s.E1*s.N3))        // M3/E1
		jac.Set(14, 8,  2*(-s.E0*s.N1 + s.E3*s.N2 - s.E2*s.N3))        // M3/E2
		jac.Set(14, 9,  2*(+s.E1*s.N1 + s.E2*s.N2 + s.E3*s.N3))        // M3/E3
		jac.Set(14, 13, 2*(s.E3*s.E1 - s.E0*s.E2))                     // M3/N1
		jac.Set(14, 14, 2*(s.E3*s.E2 + s.E0*s.E1))                     // M3/N2
		jac.Set(14, 15, 2*(s.E3*s.E3 + s.E0*s.E0 - 0.5))               // M3/N3
		jac.Set(14, 31, 1)                                             // M3/L3
	*/

	return
}

var KalmanJSONConfig = ""
