// Package ahrs implements a Kalman filter for determining aircraft kinematic state
// based on inputs from IMU and GPS
package ahrs

import (
	"fmt"
	"math"

	"github.com/skelterjohn/go.matrix"
)

// State holds the complete information describing the state of the aircraft
// Order within State also defines order in the matrices below
type State struct {
	U1, U2, U3     float64            // Vector for airspeed, aircraft (accelerated) frame
	E0, E1, E2, E3 float64            // Quaternion rotating aircraft to earth frame
	V1, V2, V3     float64            // Vector describing windspeed, latlong axes, earth (inertial) frame
	M1, M2, M3     float64            // Vector describing reference magnetometer direction, earth (inertial) frame
	T              uint32             // Timestamp when last updated
	M              matrix.DenseMatrix // Covariance matrix of state uncertainty, same order as above vars

	F0, F1, F2, F3 float64 // Calibration quaternion describing roll, pitch and heading biases due to placement of stratux, aircraft frame
	f11, f12, f13,
	f21, f22, f23,
	f31, f32, f33 float64 // After calibration, these are quaterion fragments for rotating stratux to level
}

// Control holds the control inputs for the Kalman filter: gyro change and accelerations
type Control struct {
	H1, H2, H3 float64 // Vector of gyro rates in roll, pitch, heading axes, aircraft (accelerated) frame
	A1, A2, A3 float64 // Vector holding accelerometer readings, g's, aircraft (accelerated) frame
	T          uint32  // Timestamp of readings
}

// Measurement holds the measurements used for updating the Kalman filter: groundspeed, true airspeed, magnetometer
// Note: airspeed and magnetometer may not be available until appropriate sensors are working
type Measurement struct { // Order here also defines order in the matrices below
	W1, W2, W3 float64 // Quaternion holding GPS speed in N/S, E/W and U/D directions, knots, latlong axes, earth (inertial) frame
	U1, U2, U3 float64 // Quaternion holding measured airspeed, knots, aircraft (accelerated) frame
	M1, M2, M3 float64 // Quaternion holding magnetometer readings, aircraft (accelerated) frame
	T          uint32  // Timestamp of GPS, airspeed and magnetometer readings
}

const (
	G = 32.1740 / 1.687810 // G is the acceleration due to gravity, kt/s
)

// X0 is a starting state: vector quantities are all 0's
var X0 = State{
	U1: 50, // Reasonable starting airspeed for GA aircraft
	E0: 1,  // Zero rotation has real part 1
	M: *matrix.Diagonal([]float64{
		100 * 100, 10 * 10, 10 * 10, // Reasonable for a GA aircraft
		1, 1, 1, 1, // Initial orientation is highly uncertain
		20 * 20, 20 * 20, 0.5 * 0.5, // Windspeed is an unknown
		1, 1, 1, // Magnetometer is an unknown
	}),
}

// VX represents process uncertainties, per second
var VX = State{
	U1: 1, U2: 0.2, U3: 0.3,
	E0: 1e-2, E1: 1e-2, E2: 1e-2, E3: 1e-2,
	V1: 0.005, V2: 0.005, V3: 0.05,
	M1: 0.005, M2: 0.005, M3: 0.005,
	T: 1000,
}

// VM represents measurement uncertainties
var VM = Measurement{
	W1: 0.5, W2: 0.5, W3: 0.5, // GPS uncertainty is small
	U1: 0.5, U2: 0.1, U3: 0.1, // Also airspeed
	M1: 0.1, M2: 0.1, M3: 0.1, // Also magnetometer
	T: 0,
}

// normalize normalizes the E quaternion in State s
func (s *State) normalize() {
	ee := math.Sqrt(s.E0*s.E0 + s.E1*s.E1 + s.E2*s.E2 + s.E3*s.E3)
	s.E0 /= ee
	s.E1 /= ee
	s.E2 /= ee
	s.E3 /= ee
}

// Calibrate performs a calibration, determining the quaternion to rotate it to
// be effectively level
func (s *State) Calibrate() {
	//TODO: Do the calibration to determine the Fi
	// Persist last known to storage
	// Initial is last known
	// If no GPS or GPS stationary, assume straight and level: Ai is down
	// If GPS speed, assume heading = track
	s.F0 = 1
	s.F1 = 0
	s.F2 = 0
	s.F3 = 0

	// Set the quaternion fragments to rotate from sensor frame into aircraft frame
	s.f11 = 2 * (+s.F0*s.F0 + s.F1*s.F1 - 0.5)
	s.f12 = 2 * (+s.F0*s.F3 + s.F1*s.F2)
	s.f13 = 2 * (-s.F0*s.F2 + s.F1*s.F3)
	s.f21 = 2 * (-s.F0*s.F3 + s.F2*s.F1)
	s.f22 = 2 * (+s.F0*s.F0 + s.F2*s.F2 - 0.5)
	s.f23 = 2 * (+s.F0*s.F1 + s.F2*s.F3)
	s.f31 = 2 * (+s.F0*s.F2 + s.F3*s.F1)
	s.f32 = 2 * (-s.F0*s.F1 + s.F3*s.F2)
	s.f33 = 2 * (+s.F0*s.F0 + s.F3*s.F3 - 0.5)
}

// Predict performs the prediction phase of the Kalman filter given the control inputs
func (s *State) Predict(c Control, n State) {
	f := s.calcJacobianState(c)
	dt := float64(c.T-s.T) / 1000

	// Apply the calibration quaternion F to rotate the stratux sensors to level
	h1 := c.H1*s.f11 + c.H2*s.f12 + c.H3*s.f13
	h2 := c.H1*s.f21 + c.H2*s.f22 + c.H3*s.f23
	h3 := c.H1*s.f31 + c.H2*s.f32 + c.H3*s.f33

	a1 := c.A1*s.f11 + c.A2*s.f12 + c.A3*s.f13
	a2 := c.A1*s.f21 + c.A2*s.f22 + c.A3*s.f23
	a3 := c.A1*s.f31 + c.A2*s.f32 + c.A3*s.f33

	s.U1 += dt * (-2*G*(s.E3*s.E1+s.E0*s.E2) - G*a1 - h3*s.U2 + h2*s.U3)
	s.U2 += dt * (-2*G*(s.E3*s.E2-s.E0*s.E1) - G*a2 - h1*s.U3 + h3*s.U1)
	s.U3 += dt * (-2*G*(s.E3*s.E3+s.E0*s.E0-0.5) - G*a3 - h2*s.U1 + h1*s.U2)

	s.E0 += 0.5 * dt * (-h1*s.E1 - h2*s.E2 - h3*s.E3)
	s.E1 += 0.5 * dt * (+h1*s.E0 + h2*s.E3 - h3*s.E2)
	s.E2 += 0.5 * dt * (-h1*s.E3 + h2*s.E0 + h3*s.E1)
	s.E3 += 0.5 * dt * (+h1*s.E2 - h2*s.E1 + h3*s.E0)
	s.normalize()

	// s.Vx and s.Mx are all unchanged

	s.T = c.T

	tf := dt / (float64(n.T) / 1000)
	nn := matrix.Diagonal([]float64{
		n.U1 * n.U1 * tf, n.U2 * n.U2 * tf, n.U3 * n.U3 * tf,
		n.E0 * n.E0 * tf, n.E1 * n.E1 * tf, n.E2 * n.E2 * tf, n.E3 * n.E3 * tf,
		n.V1 * n.V1 * tf, n.V2 * n.V2 * tf, n.V3 * n.V3 * tf,
		n.M1 * n.M1 * tf, n.M2 * n.M2 * tf, n.M3 * n.M3 * tf,
	})
	s.M = *matrix.Sum(matrix.Product(&f, matrix.Product(&s.M, f.Transpose())), nn)
}

// Update applies the Kalman filter corrections given the measurements
func (s *State) Update(m Measurement, n Measurement) {
	//TODO: this
	z := s.predictMeasurement()
	y := matrix.MakeDenseMatrix([]float64{
		m.W1 - z.W1, m.W2 - z.W2, m.W3 - z.W3,
		m.U1 - z.U1, m.U2 - z.U2, m.U3 - z.U3,
		m.M1 - z.M1, m.M2 - z.M2, m.M3 - z.M3,
	}, 9, 1)
	h := s.calcJacobianMeasurement()
	nn := matrix.Diagonal([]float64{
		n.W1 * n.W1, n.W2 * n.W2, n.W3 * n.W3,
		n.U1 * n.U1, n.U2 * n.U2, n.U3 * n.U3,
		n.M1 * n.M1, n.M2 * n.M2, n.M3 * n.M3,
	})
	ss := *matrix.Sum(matrix.Product(&h, matrix.Product(&s.M, h.Transpose())), nn)
	m2, err := ss.Inverse()
	if err != nil {
		fmt.Println("Can't invert Kalman gain matrix")
	}
	kk := matrix.Product(&s.M, matrix.Product(h.Transpose(), m2))
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
	s.M1 += su.Get(10, 0)
	s.M2 += su.Get(11, 0)
	s.M3 += su.Get(12, 0)
	s.T = m.T
	s.M = *matrix.Product(matrix.Difference(matrix.Eye(13), matrix.Product(kk, &h)), &s.M)
}

func (s *State) predictMeasurement() Measurement {
	var m Measurement

	m.W1 = s.V1 +
		2*s.U1*(s.E1*s.E1+s.E0*s.E0-0.5) +
		2*s.U2*(s.E1*s.E2+s.E0*s.E3) +
		2*s.U3*(s.E1*s.E3-s.E0*s.E2)
	m.W2 = s.V2 +
		2*s.U1*(s.E2*s.E1-s.E0*s.E3) +
		2*s.U2*(s.E2*s.E2+s.E0*s.E0-0.5) +
		2*s.U3*(s.E2*s.E3+s.E0*s.E1)
	m.W3 = s.V3 +
		2*s.U1*(s.E3*s.E1+s.E0*s.E2) +
		2*s.U2*(s.E3*s.E2-s.E0*s.E1) +
		2*s.U3*(s.E3*s.E3+s.E0*s.E0-0.5)

	m.U1 = s.U1
	m.U2 = s.U2
	m.U3 = s.U3

	m.M1 = 2*s.M1*(s.E1*s.E1+s.E0*s.E0-0.5) +
		2*s.M2*(s.E1*s.E2-s.E0*s.E3) +
		2*s.M3*(s.E1*s.E3+s.E0*s.E2)
	m.M2 = 2*s.M1*(s.E2*s.E1+s.E0*s.E3) +
		2*s.M2*(s.E2*s.E2+s.E0*s.E0-0.5) +
		2*s.M3*(s.E2*s.E3-s.E0*s.E1)
	m.M3 = 2*s.M1*(s.E3*s.E1-s.E0*s.E2) +
		2*s.M2*(s.E3*s.E2+s.E0*s.E1) +
		2*s.M3*(s.E3*s.E3+s.E0*s.E0-0.5)

	return m
}

func (s *State) calcJacobianState(c Control) matrix.DenseMatrix {
	dt := float64(c.T-s.T) / 1000
	data := make([][]float64, 13)
	for i := 0; i < 13; i++ {
		data[i] = make([]float64, 13)
	}

	// Apply the calibration quaternion F to rotate the stratux sensors to level
	h1 := c.H1*s.f11 + c.H2*s.f12 + c.H3*s.f13
	h2 := c.H1*s.f21 + c.H2*s.f22 + c.H3*s.f23
	h3 := c.H1*s.f31 + c.H2*s.f32 + c.H3*s.f33

	data[0][0] = 1                                         // U1,U1
	data[0][1] = -3*dt  // U1,U2
	data[0][2] = +h2*dt // U1,U3
	data[0][3] = -2*G*s.E2*dt  // U1/E0
	data[0][4] = -2*G*s.E3*dt   // U1/E1
	data[0][5] = -2*G*s.E0*dt              // U1/E2
	data[0][6] = -2*G*s.E1*dt               // U1/E3
	data[1][0] = +h3*dt // U2/U1
	data[1][1] = 1                                         // U2/U2
	data[1][2] = -h1*dt  // U2/U3
	data[1][3] = +2*G*s.E1*dt   // U2/E0
	data[1][4] = +2*G*s.E0*dt               // U2/E1
	data[1][5] = -2*G*s.E3*dt   // U2/E2
	data[1][6] = -2*G*s.E2*dt               // U2/E3
	data[2][0] = -h2*dt  // U3/U1
	data[2][1] = +h1*dt // U3/U2
	data[2][2] = 1                                         // U3/U3
	data[2][3] = -4*G*s.E0*dt   // U3/E0
	data[2][4] = -4*G*s.E1*dt              // U3/E1
	data[2][5] = -4*G*s.E2*dt              // U3/E2
	data[2][6] = -4*G*s.E3*dt   // U3/E3
	data[3][3] = 1                                         // E0/E0
	data[3][4] = -0.5*dt*h1	// U0/E1
	data[3][5] = -0.5*dt*h2	// U0/E2
	data[3][6] = -0.5*dt*h3	// U0/E3
	data[4][3] = +0.5*dt*h1	// U0/E1
	data[4][4] = 1                                         // E0/E0
	data[4][5] = -0.5*dt*h3	// U0/E3
	data[4][6] = +0.5*dt*h2	// U0/E2
	data[5][3] = +0.5*dt*h2	// U0/E2
	data[5][4] = +0.5*dt*h3	// U0/E3
	data[5][5] = 1                                         // E0/E0
	data[5][6] = -0.5*dt*h1	// U0/E1
	data[6][3] = +0.5*dt*h3	// U0/E3
	data[6][4] = -0.5*dt*h2	// U0/E2
	data[6][5] = +0.5*dt*h1	// U0/E1
	data[6][6] = 1                                         // E0/E0
	data[7][7] = 1                                         // V1/V1
	data[8][8] = 1                                         // V2/V2
	data[9][9] = 1                                         // V3/V3
	data[10][10] = 1                                       // M1/M1
	data[11][11] = 1                                       // M2/M2
	data[12][12] = 1                                       // M3/M3

	ff := *matrix.MakeDenseMatrixStacked(data)
	return ff
}

func (s *State) calcJacobianMeasurement() matrix.DenseMatrix {
	data := make([][]float64, 9)
	for i := 0; i < 9; i++ {
		data[i] = make([]float64, 13)
	}

	data[0][0] = 2*(s.E1*s.E1+s.E0*s.E0-0.5)                    // W1/U1
	data[0][1] = 2*(s.E1*s.E2+s.E0*s.E3)                        // W1/U2
	data[0][2] = 2*(s.E1*s.E3-s.E0*s.E2)                        // W1/U3
	data[0][3] = 2*(+2*s.E0*s.U1 + s.E3*s.U2 - s.E2*s.U3)       // W1/E0
	data[0][4] = 2*(+2*s.E1*s.U1 + s.E2*s.U2 + s.E3*s.U3)       // W1/E1
	data[0][5] = 2*(              s.E1*s.U2 - s.E0*s.U3)        // W1/E2
	data[0][6] = 2*(              s.E0*s.U2 + s.E1*s.U3)        // W1/E3
	data[0][7] = 1                                              // W1/V1
	data[1][0] = 2*(s.E2*s.E1 - s.E0*s.E3)                      // W2/U1
	data[1][1] = 2*(s.E2*s.E2 + s.E0*s.E0 - 0.5)		    // W2/U2
	data[1][2] = 2*(s.E2*s.E3 + s.E0*s.E1)                      // W2/U3
	data[1][3] = 2*(-s.E3*s.U1 + 2*s.E0*s.U2 + s.E1*s.U3)       // W2/E0
	data[1][4] = 2*(+s.E2*s.U1               + s.E0*s.U3)       // W2/E1
	data[1][5] = 2*(+s.E1*s.U1 + 2*s.E2*s.U2 + s.E3*s.U3)       // W2/E2
	data[1][6] = 2*(-s.E0*s.U1               + s.E2*s.U3)       // W2/E3
	data[1][8] = 1                                              // W2/V2
	data[2][0] = 2*(s.E3*s.E1 + s.E0*s.E2)                      // W3/U1
	data[2][1] = 2*(s.E3*s.E2 - s.E0*s.E1)                      // W3/U2
	data[2][2] = 2*(s.E3*s.E3 + s.E0*s.E0 - 0.5)                // W3/U3
	data[2][3] = 2*(+s.E2*s.U1 - s.E1*s.U2 + 2*s.E0*s.U3)       // W3/E0
	data[2][4] = 2*(+s.E3*s.U1 - s.E0*s.U2)                     // W3/E1
	data[2][5] = 2*(+s.E0*s.U1 + s.E3*s.U2)                     // W3/E2
	data[2][6] = 2*(+s.E1*s.U1 + s.E2*s.U2 + 2*s.E3*s.U3)       // W3/E3
	data[2][9] = 1                                              // W3/V3
	data[3][0] = 1                                              // U1/U1
	data[4][1] = 1                                              // U2/U2
	data[5][2] = 1                                              // U3/U3

	data[6][3] = 2*(+2*s.E0*s.M1 - s.E3*s.M2 + s.E2*s.M3)       // M1/E0
	data[6][4] = 2*(+2*s.E1*s.M1 + s.E2*s.M2 + s.E3*s.M3)       // M1/E1
	data[6][5] = 2*(               s.E1*s.M2 + s.E0*s.M3)       // M1/E2
	data[6][6] = 2*(             - s.E0*s.M2 + s.E1*s.M3)       // M1/E3
	data[6][10] = 2*(s.E1*s.E1+s.E0*s.E0-0.5)                   // M1/M1
	data[6][11] = 2*(s.E1*s.E2-s.E0*s.E3)                       // M1/M2
	data[6][12] = 2*(s.E1*s.E3+s.E0*s.E2)                       // M1/M3
	data[7][3] =  2*(+s.E3*s.M1 + 2*s.E0*s.M2 - s.E1*s.M3)      // M2/E0
	data[7][4] =  2*(+s.E2*s.M1               - s.E0*s.M3)      // M2/E1
	data[7][5] =  2*(+s.E1*s.M1 + 2*s.E2*s.M2 + s.E3*s.M3)      // M2/E2
	data[7][6] =  2*(+s.E0*s.M1               + s.E2*s.M3)      // M2/E3
	data[7][10] = 2*(s.E2*s.E1 + s.E0*s.E3)                     // M2/M1
	data[7][11] = 2*(s.E2*s.E2 + s.E0*s.E0 - 0.5)               // M2/M2
	data[7][12] = 2*(s.E2*s.E3 - s.E0*s.E1)                     // M2/M3
	data[8][3] =  2*(-s.E2*s.M1 + s.E1*s.M2 + 2*s.E0*s.M3)      // M3/E0
	data[8][4] =  2*(+s.E3*s.M1 + s.E0*s.M2)                    // M3/E1
	data[8][5] =  2*(-s.E0*s.M1 + s.E3*s.M2)                    // M3/E2
	data[8][6] =  2*(+s.E1*s.M1 + s.E2*s.M2 + 2*s.E3*s.M3)      // M3/E3
	data[8][10] = 2*(s.E3*s.E1 - s.E0*s.E2)                     // M3/M1
	data[8][11] = 2*(s.E3*s.E2 + s.E0*s.E1)                     // M3/M2
	data[8][12] = 3*(s.E3*s.E3 + s.E0*s.E0 - 0.5)               // M3/M3

	hh := *matrix.MakeDenseMatrixStacked(data)
	return hh
}
