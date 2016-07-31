// Package ahrs implements a Kalman filter for determining aircraft kinematic state
// based on inputs from IMU and GPS
package ahrs

import (
	"github.com/skelterjohn/go.matrix"
	"log"
	"math"
)

const (
	G = 32.1740 / 1.687810  // G is the acceleration due to gravity, kt/s
)

// State holds the complete information describing the state of the aircraft
// Aircraft frame is noninertial: 1 is to nose; 2 is to left wing; 3 is up
// Earth frame is inertial: 1 is east; 2 is north; 3 is up
// Sensor frame is fixed within aircraft frame, so noninertial, rotated
type State struct {
	U1, U2, U3    	float64             // Vector for airspeed, aircraft frame
	Z1, Z2, Z3      float64             // Vector for rate of change of airspeed, aircraft frame
	E0, E1, E2, E3	float64             // Quaternion rotating earth frame to aircraft frame
	H1, H2, H3      float64             // Vector for gyro rates, aircraft frame
	N1, N2, N3    	float64             // Vector for earth's magnetic field, earth (inertial) frame

	V1, V2, V3    	float64             // (Bias) Vector for windspeed, earth frame
	C1, C2, C3      float64             // Bias vector for accelerometer, sensor frame
	F0, F1, F2, F3	float64             // (Bias) quaternion rotating sensor frame to aircraft frame
	D1, D2, D3      float64             // Bias vector for gyro rates, sensor frame
	L1, L2, L3      float64             // Bias vector for magnetometer direction, sensor frame

	T             	float64             // Time when state last updated

	M             	*matrix.DenseMatrix // Covariance matrix of state uncertainty, same order as above vars:
	N               *matrix.DenseMatrix // Covariance matrix of state noise per unit time
                                            // U, Z, E, H, N,
                                            // V, C, F, D, L

	e11, e12, e13   float64             // cached quaternion fragment
	e21, e22, e23   float64             // cached quaternion fragment
	e31, e32, e33   float64             // cached quaternion fragment

	f11, f12, f13   float64             // cached quaternion fragment
	f21, f22, f23   float64             // cached quaternion fragment
	f31, f32, f33   float64             // cached quaternion fragment
}

// Measurement holds the measurements used for updating the Kalman filter: groundspeed, true airspeed, magnetometer
// Note: airspeed and magnetometer may not be available until appropriate sensors are working
type Measurement struct {                      // Order here also defines order in the matrices below
	UValid, WValid, SValid, MValid bool    // Do we have valid airspeed, GPS, accel/gyro, and magnetometer readings?
	// U, W, A, B, M
	U1, U2, U3                     float64 // Quaternion holding measured airspeed, knots, aircraft (accelerated) frame
	W1, W2, W3                     float64 // Quaternion holding GPS speed in N/S, E/W and U/D directions, knots, latlong axes, earth (inertial) frame
	A1, A2, A3                     float64 // Vector holding accelerometer readings, g's, aircraft (accelerated) frame
	B1, B2, B3                     float64 // Vector of gyro rates in roll, pitch, heading axes, aircraft (accelerated) frame
	M1, M2, M3                     float64 // Quaternion holding magnetometer readings, aircraft (accelerated) frame
	T                              float64 // Timestamp of GPS, airspeed and magnetometer readings

	M                              *matrix.DenseMatrix // Measurement noise covariance
}

// normalize normalizes the E & F quaternions in State s
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

	// eij rotates earth frame i component into aircraft frame j component
	s.e11 = 2 * (+s.E0 * s.E0 + s.E1 * s.E1 - 0.5)
	s.e12 = 2 * (+s.E0 * s.E3 + s.E1 * s.E2)
	s.e13 = 2 * (-s.E0 * s.E2 + s.E1 * s.E3)
	s.e21 = 2 * (-s.E0 * s.E3 + s.E2 * s.E1)
	s.e22 = 2 * (+s.E0 * s.E0 + s.E2 * s.E2 - 0.5)
	s.e23 = 2 * (+s.E0 * s.E1 + s.E2 * s.E3)
	s.e31 = 2 * (+s.E0 * s.E2 + s.E3 * s.E1)
	s.e32 = 2 * (-s.E0 * s.E1 + s.E3 * s.E2)
	s.e33 = 2 * (+s.E0 * s.E0 + s.E3 * s.E3 - 0.5)

	// fij rotates sensor frame i component into aircraft frame j component
	s.f11 = 2 * (+s.F0 * s.F0 + s.F1 * s.F1 - 0.5)
	s.f12 = 2 * (+s.F0 * s.F3 + s.F1 * s.F2)
	s.f13 = 2 * (-s.F0 * s.F2 + s.F1 * s.F3)
	s.f21 = 2 * (-s.F0 * s.F3 + s.F2 * s.F1)
	s.f22 = 2 * (+s.F0 * s.F0 + s.F2 * s.F2 - 0.5)
	s.f23 = 2 * (+s.F0 * s.F1 + s.F2 * s.F3)
	s.f31 = 2 * (+s.F0 * s.F2 + s.F3 * s.F1)
	s.f32 = 2 * (-s.F0 * s.F1 + s.F3 * s.F2)
	s.f33 = 2 * (+s.F0 * s.F0 + s.F3 * s.F3 - 0.5)

}

// Initialize the state at the start of the Kalman filter, based on current measurements
func Initialize(m *Measurement) (s *State) {
	s = new(State)

	// U, Z, E, H, N,
	// V, C, F, D, L
	s.M = matrix.Diagonal([]float64{
		50, 5, 5,           // U*3
		5, 2, 2,            // Z*3
		0.5, 0.5, 0.5, 0.5, // E*4
		5, 5, 5,            // H*3
		0.5, 0.5, 0.5,      // N*3
		20, 20, 2,          // V*3
		0.05, 0.05, 0.05,   // C*3
		0.5, 0.5, 0.5, 0.5, // F*4
		0.1, 0.1, 0.1,      // D*4
		0.1, 0.1, 0.1,      // L*4
	})
	s.M = matrix.Product(s.M, s.M)

	s.N = matrix.Diagonal([]float64{
		1, 0.1, 0.1,                            // U*3
		0.2, 0.2, 0.2,                          // Z*3
		0.02, 0.02, 0.02, 0.02,                 // E*4
		5, 5, 5,                                // H*3
		0.01, 0.01, 0.01,                       // N*3
		0.5, 0.5, 0.5,                          // V*3
		0.01/60, 0.01/60, 0.01/60,              // C*3
		0.001/60, 0.001/60, 0.001/60, 0.001/60, // F*4
		0.1/60, 0.1/60, 0.1/60,                 // D*3
		0.01/60, 0.01/60, 0.01/60,              // L*3
	})
	s.N = matrix.Product(s.N, s.N)

	//TODO westphae: for now just treat the case !m.UValid; if we have U, we can do a lot more!

	// Best guess at initial airspeed is initial groundspeed
	if m.WValid {
		s.U1 = math.Hypot(m.W1, m.W2)
		s.M.Set(0, 0, 10*10) // Our estimate of airspeed is better
		s.M.Set(16, 16, 50) // Matching uncertainty of windspeed
		s.M.Set(17, 17, 50) // Matching uncertainty of windspeed
	}

	// Best guess at initial heading is initial track
	if m.WValid && s.U1 > 5 {
		// Simplified half-angle formulae
		s.E0, s.E3 = math.Sqrt((s.U1 + m.W1) / (2 * s.U1)), math.Sqrt((s.U1 - m.W1) / (2 * s.U1))
		if m.W2 > 0 {
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
	}

	return
}

// Predict performs the prediction phase of the Kalman filter
func (s *State) Predict(t float64) {
	f := s.calcJacobianState(t)
	dt := t - s.T

	s.U1 += dt * s.Z1
	s.U2 += dt * s.Z2
	s.U3 += dt * s.Z3

	s.E0 += 0.5 * dt * (-s.H1*s.E1 - s.H2*s.E2 - s.H3*s.E3)
	s.E1 += 0.5 * dt * (+s.H1*s.E0 + s.H2*s.E3 - s.H3*s.E2)
	s.E2 += 0.5 * dt * (-s.H1*s.E3 + s.H2*s.E0 + s.H3*s.E1)
	s.E3 += 0.5 * dt * (+s.H1*s.E2 - s.H2*s.E1 + s.H3*s.E0)
	s.normalize()

	// All other state vectors are unchanged

	s.T = t

	s.M = matrix.Sum(matrix.Product(f, matrix.Product(s.M, f.Transpose())), matrix.Scaled(s.N, dt))
	// U, Z, E, H, N,
	// V, C, F, D, L
}

// Update applies the Kalman filter corrections given the measurements
func (s *State) Update(m *Measurement) {
	z := s.PredictMeasurement()
	y := matrix.MakeDenseMatrix([]float64{
		m.U1 - z.U1, m.U2 - z.U2, m.U3 - z.U3,
		m.W1 - z.W1, m.W2 - z.W2, m.W3 - z.W3,
		m.A1 - z.A1, m.A2 - z.A2, m.A3 - z.A3,
		m.B1 - z.B1, m.B2 - z.B2, m.B3 - z.B3,
		m.M1 - z.M1, m.M2 - z.M2, m.M3 - z.M3,
	}, 15, 1)
	h := s.calcJacobianMeasurement()

	// U, W, A, B, M
	if m.UValid {
		m.M.Set(0, 0, 2)
	} else {
		y.Set(0, 0, 0)
		m.M.Set(0, 0, 1e9)
	}
	// U2, U3 are just here to bias toward coordinated flight
	m.M.Set(1, 1, 25)
	m.M.Set(2, 2, 25)

	if m.WValid {
		m.M.Set(3, 0, 0.2)
		m.M.Set(4, 1, 0.2)
		m.M.Set(5, 2, 0.2)
	} else {
		y.Set(3, 0, 0)
		y.Set(4, 0, 0)
		y.Set(5, 0, 0)
		m.M.Set(3, 3, 1e9)
		m.M.Set(4, 4, 1e9)
		m.M.Set(5, 5, 1e9)
	}

	if m.SValid {
		m.M.Set( 6,  6, 0.2)
		m.M.Set( 7,  7, 0.2)
		m.M.Set( 8,  8, 0.2)
		m.M.Set( 9,  9,   1)
		m.M.Set(10, 10,   1)
		m.M.Set(11, 11,   1)
	} else {
		y.Set( 6, 0, 0)
		y.Set( 7, 0, 0)
		y.Set( 8, 0, 0)
		y.Set( 9, 0, 0)
		y.Set(10, 0, 0)
		y.Set(11, 0, 0)
		m.M.Set( 6,  6, 1e9)
		m.M.Set( 7,  7, 1e9)
		m.M.Set( 8,  8, 1e9)
		m.M.Set( 9,  9, 1e9)
		m.M.Set(10, 10, 1e9)
		m.M.Set(11, 11, 1e9)
	}

	if m.MValid {
		m.M.Set(12, 12, 0.1) //TODO westphae: set all magnetometer uncertainties properly
		m.M.Set(13, 13, 0.1)
		m.M.Set(14, 14, 0.1)
	} else {
		y.Set(12, 0, 0)
		y.Set(13, 0, 0)
		y.Set(14, 0, 0)
		m.M.Set(12, 12, 1e9)
		m.M.Set(13, 13, 1e9)
		m.M.Set(14, 14, 1e9)
	}
	ss := *matrix.Sum(matrix.Product(h, matrix.Product(s.M, h.Transpose())), m.M)

	m2, err := ss.Inverse()
	if err != nil {
		log.Println("AHRS: Can't invert Kalman gain matrix")
	}
	kk := matrix.Product(s.M, matrix.Product(h.Transpose(), m2))
	su := matrix.Product(kk, y)
	s.U1 += su.Get(0, 0)
	s.U2 += su.Get(1, 0)
	s.U3 += su.Get(2, 0)
	s.E0 += su.Get(3, 0)
	s.E1 += su.Get(4, 0)
	s.E2 += su.Get(5, 0)
	s.E3 += su.Get(6, 0)
	s.normalize()
	s.V1 += su.Get(7, 0)
	s.V2 += su.Get(8, 0)
	s.V3 += su.Get(9, 0)
	s.N1 += su.Get(10, 0)
	s.N2 += su.Get(11, 0)
	s.N3 += su.Get(12, 0)
	s.T = m.T
	s.M = matrix.Product(matrix.Difference(matrix.Eye(13), matrix.Product(kk, h)), s.M)
}

func (s *State) PredictMeasurement() (m *Measurement) {
	m = new(Measurement)

	m.WValid = true
	m.W1 = s.e11*s.U1 + s.e12*s.U2 + s.e13*s.U3 + s.V1
	m.W2 = s.e21*s.U1 + s.e22*s.U2 + s.e23*s.U3 + s.V2
	m.W3 = s.e31*s.U1 + s.e32*s.U2 + s.e33*s.U3 + s.V3

	m.UValid = true
	m.U1 = s.U1
	m.U2 = s.U2
	m.U3 = s.U3

	m.SValid = true
	// Include pseudoforces from non-inertial frame!  Why we see "contamination" of accel from gyro
	a1 := -(s.Z1 + s.H2*s.U3 - s.H3*s.U2)/G - s.e31
	a2 := -(s.Z2 + s.H3*s.U1 - s.H1*s.U3)/G - s.e32
	a3 := -(s.Z3 + s.H1*s.U2 - s.H2*s.U1)/G - s.e33
	m.A1 = a1*s.f11 + a2*s.f21 + a3*s.f31 + s.C1
	m.A2 = a1*s.f12 + a2*s.f22 + a3*s.f32 + s.C2
	m.A3 = a1*s.f13 + a2*s.f23 + a3*s.f33 + s.C3

	m.B1 = s.H1*s.f11 + s.H2*s.f21 + s.H3*s.f31 + s.D1
	m.B2 = s.H1*s.f12 + s.H2*s.f22 + s.H3*s.f32 + s.D2
	m.B3 = s.H1*s.f13 + s.H2*s.f23 + s.H3*s.f33 + s.D3

	m.MValid = true
	m1 :=  s.N1*s.e11 + s.N2*s.e21 + s.N3*s.e31 + s.L1
	m2 :=  s.N1*s.e12 + s.N2*s.e22 + s.N3*s.e32 + s.L2
	m3 :=  s.N1*s.e13 + s.N2*s.e23 + s.N3*s.e33 + s.L3
	m.M1 = s.f11*m1 + s.f21*m2 + s.f31*m3 + s.L1
	m.M2 = s.f12*m1 + s.f22*m2 + s.f32*m3 + s.L2
	m.M3 = s.f13*m1 + s.f23*m2 + s.f33*m3 + s.L3

	m.T = s.T

	return
}

func (s *State) calcJacobianState(t float64) (jac *matrix.DenseMatrix) {
	dt := t-s.T

	jac = matrix.Eye(32)
	// U*3, Z*3, E*4, H*3, N*3,
	// V*3, C*3, F*4, D*3, L*3

	jac.Set(0, 3, dt)            // U1/Z1
	jac.Set(1, 4, dt)            // U2/Z2
	jac.Set(2, 5, dt)            // U3/Z3

	jac.Set(6, 7, -0.5*dt*s.H1)  // E0/E1
	jac.Set(6, 8, -0.5*dt*s.H2)  // E0/E2
	jac.Set(6, 9, -0.5*dt*s.H3)  // E0/E3

	jac.Set(7, 6, +0.5*dt*s.H1)  // E1/E0
	jac.Set(7, 8, -0.5*dt*s.H3)  // E1/E2
	jac.Set(7, 9, +0.5*dt*s.H2)  // E1/E3

	jac.Set(8, 6, +0.5*dt*s.H2)  // E2/E0
	jac.Set(8, 7, +0.5*dt*s.H3)  // E2/E1
	jac.Set(8, 9, -0.5*dt*s.H1)  // E2/E3

	jac.Set(9, 6, +0.5*dt*s.H3)  // E3/E0
	jac.Set(9, 7, -0.5*dt*s.H2)  // E3/E1
	jac.Set(9, 8, +0.5*dt*s.H1)  // E3/E2

	return
}

func (s *State) calcJacobianMeasurement() (jac *matrix.DenseMatrix) {

	jac = new(matrix.DenseMatrix, 15, 32)
	// U*3, Z*3, E*4, H*3, N*3,
	// V*3, C*3, F*4, D*3, L*3
	// U*3, W*3, A*3, B*3, M*3

	jac.Set(0, 0, 1)                                              // U1/U1
	jac.Set(1, 1, 1)                                              // U2/U2
	jac.Set(2, 2, 1)                                              // U3/U3

	jac.Set(3, 0, s.e11)                                            // W1/U1
	jac.Set(3, 1, s.e12)                                            // W1/U2
	jac.Set(3, 2, s.e13)                                            // W1/U3
	jac.Set(3, 6, 2*(+s.E0*s.U1 + s.E3*s.U2 - s.E2*s.U3))         // W1/E0
	jac.Set(3, 7, 2*(+s.E1*s.U1 + s.E2*s.U2 + s.E3*s.U3))         // W1/E1
	jac.Set(3, 8, 2*(-s.E2*s.U1 + s.E1*s.U2 - s.E0*s.U3))         // W1/E2
	jac.Set(3, 9, 2*(-s.E3*s.U1 + s.E0*s.U2 + s.E1*s.U3))         // W1/E3
	jac.Set(3, 16, 1)                                             // W1/V1

	jac.Set(4, 0, s.e21)                                            // W2/U1
	jac.Set(4, 1, s.e22)                                            // W2/U2
	jac.Set(4, 2, s.e23)                                            // W2/U3
	jac.Set(4, 6, 2*(-s.E3*s.U1 + s.E0*s.U2 + s.E1*s.U3))         // W2/E0
	jac.Set(4, 7, 2*(+s.E2*s.U1 - s.E1*s.U2 + s.E0*s.U3))         // W2/E1
	jac.Set(4, 8, 2*(+s.E1*s.U1 + s.E2*s.U2 + s.E3*s.U3))         // W2/E2
	jac.Set(4, 9, 2*(-s.E0*s.U1 - s.E3*s.U2 + s.E2*s.U3))         // W2/E3
	jac.Set(4, 17, 1)                                             // W2/V2

	jac.Set(5, 0, s.e31)                                            // W3/U1
	jac.Set(5, 1, s.e32)                                            // W3/U2
	jac.Set(5, 2, s.e33)                                            // W3/U3
	jac.Set(5, 6, 2*(+s.E2*s.U1 - s.E1*s.U2 + s.E0*s.U3))         // W3/E0
	jac.Set(5, 7, 2*(+s.E3*s.U1 - s.E0*s.U2 - s.E1*s.U3))         // W3/E1
	jac.Set(5, 8, 2*(+s.E0*s.U1 + s.E3*s.U2 - s.E2*s.U3))         // W3/E2
	jac.Set(5, 9, 2*(+s.E1*s.U1 + s.E2*s.U2 + s.E3*s.U3))         // W3/E3
	jac.Set(5, 18, 1)                                             // W3/V3

	a1 := -(s.Z1 + s.H2*s.U3 - s.H3*s.U2)/G - s.e31
	a2 := -(s.Z2 + s.H3*s.U1 - s.H1*s.U3)/G - s.e32
	a3 := -(s.Z3 + s.H1*s.U2 - s.H2*s.U1)/G - s.e33

	//m.A1 = a1*f11 + a2*f21 + a3*f31 + s.C1
	jac.Set(6, 0, (s.H2*s.f31 - s.H3*s.f21)/G)                        // A1/U1
	jac.Set(6, 1, (s.H3*s.f11 - s.H1*s.f31)/G)                        // A1/U2
	jac.Set(6, 2, (s.H1*s.f21 - s.H2*s.f11)/G)                        // A1/U3
	jac.Set(6, 3, -s.f11)                                           // A1/Z1
	jac.Set(6, 4, -s.f21)                                           // A1/Z2
	jac.Set(6, 5, -s.f31)                                           // A1/Z3
	jac.Set(6, 6, -2*(s.E2*s.f11 - s.E1*s.f21 + s.E0*s.f31))            // A1/E0
	jac.Set(6, 7, -2*(s.E3*s.f11 - s.E0*s.f21 - s.E1*s.f31))            // A1/E1
	jac.Set(6, 8, -2*(s.E0*s.f11 + s.E3*s.f21 - s.E2*s.f31))            // A1/E2
	jac.Set(6, 9, -2*(s.E1*s.f11 + s.E2*s.f21 + s.E3*s.f31))            // A1/E3
	jac.Set(6, 10, (s.U3*s.f21 - s.U2*s.f31)/G)                       // A1/H1
	jac.Set(6, 11, (s.U1*s.f31 - s.U3*s.f11)/G)                       // A1/H2
	jac.Set(6, 12, (s.U2*s.f11 - s.U1*s.f21)/G)                       // A1/H3
	jac.Set(6, 19, 1)                                             // A1/C1
	jac.Set(6, 22, 2*(+s.F0*a1 - s.F3*a2 + s.F2*a3))              // A1/F0
	jac.Set(6, 23, 2*(+s.F1*a1 + s.F2*a2 + s.F3*a3))              // A1/F1
	jac.Set(6, 24, 2*(-s.F2*a1 + s.F1*a2 + s.F0*a3))              // A1/F2
	jac.Set(6, 25, 2*(-s.F3*a1 - s.F0*a2 + s.F1*a3))              // A1/F3

	//m.A2 = a1*f12 + a2*f22 + a3*f32 + s.C2
	jac.Set(7, 0, (s.H2*s.f32 - s.H3*s.f22)/G)                        // A2/U1
	jac.Set(7, 1, (s.H3*s.f12 - s.H1*s.f32)/G)                        // A2/U2
	jac.Set(7, 2, (s.H1*s.f22 - s.H2*s.f12)/G)                        // A2/U3
	jac.Set(7, 3, -s.f12)                                           // A2/Z1
	jac.Set(7, 4, -s.f22)                                           // A2/Z2
	jac.Set(7, 5, -s.f32)                                           // A2/Z3
	jac.Set(7, 6, -2*(s.E2*s.f12 - s.E1*s.f22 + s.E0*s.f32))            // A2/E0
	jac.Set(7, 7, -2*(s.E3*s.f12 - s.E0*s.f22 - s.E1*s.f32))            // A2/E1
	jac.Set(7, 8, -2*(s.E0*s.f12 + s.E3*s.f22 - s.E2*s.f32))            // A2/E2
	jac.Set(7, 9, -2*(s.E1*s.f12 + s.E2*s.f22 + s.E3*s.f32))            // A2/E3
	jac.Set(7, 10, (s.U3*s.f22 - s.U2*s.f32)/G)                       // A2/H1
	jac.Set(7, 11, (s.U1*s.f32 - s.U3*s.f12)/G)                       // A2/H2
	jac.Set(7, 12, (s.U2*s.f12 - s.U1*s.f22)/G)                       // A2/H3
	jac.Set(7, 20, 1)                                             // A2/C2
	jac.Set(7, 22, 2*(+s.F3*a1 + s.F0*a2 - s.F1*a3))              // A2/F0
	jac.Set(7, 23, 2*(+s.F2*a1 - s.F1*a2 - s.F0*a3))              // A2/F1
	jac.Set(7, 24, 2*(+s.F1*a1 + s.F2*a2 + s.F3*a3))              // A2/F2
	jac.Set(7, 25, 2*(+s.F0*a1 - s.F3*a2 + s.F2*a3))              // A2/F3

	//m.A3 = a1*f13 + a2*f23 + a3*f33 + s.C3
	jac.Set(8, 0, (s.H2*s.f33 - s.H3*s.f23)/G)                        // A3/U1
	jac.Set(8, 1, (s.H3*s.f13 - s.H1*s.f33)/G)                        // A3/U2
	jac.Set(8, 2, (s.H1*s.f23 - s.H2*s.f13)/G)                        // A3/U3
	jac.Set(8, 3, -s.f13)                                           // A3/Z1
	jac.Set(8, 4, -s.f23)                                           // A3/Z2
	jac.Set(8, 5, -s.f33)                                           // A3/Z3
	jac.Set(8, 6, -2*(s.E2*s.f13 - s.E1*s.f23 + s.E0*s.f33))            // A3/E0
	jac.Set(8, 7, -2*(s.E3*s.f13 - s.E0*s.f23 - s.E1*s.f33))            // A3/E1
	jac.Set(8, 8, -2*(s.E0*s.f13 + s.E3*s.f23 - s.E2*s.f33))            // A3/E2
	jac.Set(8, 9, -2*(s.E1*s.f13 + s.E2*s.f23 + s.E3*s.f33))            // A3/E3
	jac.Set(8, 10, (s.U3*s.f23 - s.U2*s.f33)/G)                       // A3/H1
	jac.Set(8, 11, (s.U1*s.f33 - s.U3*s.f13)/G)                       // A3/H2
	jac.Set(8, 12, (s.U2*s.f13 - s.U1*s.f23)/G)                       // A3/H3
	jac.Set(8, 21, 1)                                             // A3/C3
	jac.Set(8, 22, 2*(-s.F2*a1 + s.F1*a2 + s.F0*a3))              // A3/F0
	jac.Set(8, 23, 2*(+s.F3*a1 + s.F0*a2 - s.F1*a3))              // A3/F1
	jac.Set(8, 24, 2*(-s.F0*a1 + s.F3*a2 - s.F2*a3))              // A3/F2
	jac.Set(8, 25, 2*(+s.F1*a1 + s.F2*a2 + s.F3*a3))              // A3/F3

	//m.B1 = s.H1*f11 + s.H2*f21 + s.H3*f31 + s.D1
	jac.Set(9, 10, s.f11)                                           // B1/H1
	jac.Set(9, 11, s.f21)                                           // B1/H2
	jac.Set(9, 12, s.f31)                                           // B1/H3
	jac.Set(9, 26, 1)                                             // B1/D1

	//m.B2 = s.H1*f12 + s.H2*f22 + s.H3*f32 + s.D2
	jac.Set(10, 10, s.f12)                                          // B2/H1
	jac.Set(10, 11, s.f22)                                          // B2/H2
	jac.Set(10, 12, s.f32)                                          // B2/H3
	jac.Set(10, 27, 1)                                            // B2/D2

	//m.B3 = s.H1*f13 + s.H2*f23 + s.H3*f33 + s.D3
	jac.Set(11, 10, s.f13)                                          // B3/H1
	jac.Set(11, 11, s.f23)                                          // B3/H2
	jac.Set(11, 12, s.f33)                                          // B3/H3
	jac.Set(11, 28, 1)                                            // B3/D3

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
