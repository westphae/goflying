// Package ahrs implements a Kalman filter for determining aircraft kinematic state
// based on inputs from IMU and GPS
package ahrs

import (
	"fmt"
	"github.com/skelterjohn/go.matrix"
	"math"
)

type State struct { // Order here also defines order in the matrices below
	Ux, Uy, Uz             float64            // True airspeed, sideslip speed and mush speed, aircraft (accelerated) frame
	Phi, Theta, Psi        float64            // Roll, pitch and heading, radians - relates aircraft to earth frame
	Phi0, Theta0, Psi0     float64            // Roll, pitch and heading biases due to placement on stratux, radians
	Vx, Vy, Vz             float64            // Windspeed in N/W, E/W and U/D directions, knots, latlong axes, earth (inertial) frame
	T                      uint32             // Timestamp when last updated
	M                      matrix.DenseMatrix // Covariance matrix of state uncertainty, same order as above vars
	sPhi, cPhi             float64            // sin(Phi-Phi0), cos(Phi-Phi0)
	sTheta, cTheta, tTheta float64            // sin(Theta-Theta0), cos(Theta-Theta0), tan(Theta-Theta0)
	sPsi, cPsi             float64            // sin(Psi-Psi0), cos(Psi-Psi0)
}

type Control struct {
	P, Q, R    float64 // Gyro change in roll, pitch, heading axes, rad/s, aircraft (accelerated) frame
	Ax, Ay, Az float64 // Accelerometer change in f/b, l/r, and u/d axes, g's, aircraft (accelerated) frame
	T          uint32  // Timestamp of readings
}

type Measurement struct { // Order here also defines order in the matrices below
	Wx, Wy, Wz float64 // GPS speed in N/S, E/W and U/D directions, knots, latlong axes, earth (inertial) frame
	Sx, Sy, Sz float64 // Measured airspeed, sideslip speed and mush speed, knots, aircraft (accelerated) frame
	Mx, My, Mz float64 // Magnetometer readings in f/b, l/r, and u/d axes, aircraft (accelerated) frame
	T          uint32  // Timestamp of GPS, airspeed and magnetometer readings
}

const (
	G  = 32.1740 / 1.687810 // Acceleration due to gravity, kt/s
	Pi = math.Pi
)

var X0 = State{ // Starting state: all 0's
	Ux: 0, Uy: 0, Uz: 0,
	Phi: 0, Theta: 0, Psi: 0,
	Phi0: 0, Theta0: 0, Psi0: 0,
	Vx: 0, Vy: 0, Vz: 0,
	T: 0,
	M: *matrix.Diagonal([]float64{
		100 * 100, 100 * 100, 10 * 10, // Reasonable for a GA aircraft
		(Pi / 20) * (Pi / 20), (Pi / 40) * (Pi / 40), (2 * Pi) * (2 * Pi), // Straight and level is most likely assumption
		(Pi / 2) * (Pi / 2), (Pi / 2) * (Pi / 2), (2 * Pi) * (2 * Pi), // We also have no idea how it's initially situated
		20 * 20, 20 * 20, 0.5 * 0.5, // Windspeed is an unknown
	}),
}

var VX = State{ // Process uncertainty, per second
	Ux: 1, Uy: 0.2, Uz: 0.3,
	Phi: Pi / 720, Theta: Pi / 720, Psi: Pi / 720,
	Phi0: 1e-6, Theta0: 1e-6, Psi0: 1e-6, // Biases by definition don't evolve (don't let it slip!)
	Vx: 0.005, Vy: 0.005, Vz: 0.05,
	T: 1000,
}

var VM = Measurement{
	Wx: 0.5, Wy: 0.5, Wz: 0.5, // GPS uncertainty is small
	Sx: 0.5, Sy: 0.1, Sz: 0.1, // Also airspeed
	Mx: 0.1, My: 0.1, Mz: 0.1, // Also magnetometer
	T: 0,
}

func (s *State) doTrig() { // Do trig calcs once to save CPU cycles
	s.sPhi = math.Sin(s.Phi - s.Phi0)
	s.cPhi = math.Cos(s.Phi - s.Phi0)
	s.sTheta = math.Sin(s.Theta - s.Theta0)
	s.cTheta = math.Cos(s.Theta - s.Theta0)
	s.tTheta = math.Tan(s.Theta - s.Theta0)
	s.sPsi = math.Sin(s.Psi - s.Psi0)
	s.cPsi = math.Cos(s.Psi - s.Psi0)
}

func (s *State) Predict(c Control, n State) { // Given a control, predict the new state
	s.doTrig()
	dt := float64(c.T-s.T) / 1000
	s.Ux += dt * (-G*c.Ax + c.R*s.Uy - c.Q*s.Uz - G*s.sTheta)
	s.Uy += dt * (-G*c.Ay + c.P*s.Uz - c.R*s.Ux - G*s.cTheta*s.sPhi)
	s.Uz += dt * (-G*c.Az + c.Q*s.Ux - c.P*s.Uy - G*s.cTheta*s.cPhi)
	s.Phi += dt * (c.P - s.tTheta*s.sPhi*c.Q - s.tTheta*s.cPhi*c.R)
	s.Theta += dt * (-s.cPhi*c.Q + s.sPhi*c.R)
	s.Psi += dt * (-s.sPhi/s.cTheta*c.Q - s.cPhi/s.cTheta*c.R)
	f := s.calcJacobianState(c)
	tf := dt / float64(n.T)
	nn := matrix.Diagonal([]float64{
		n.Ux * n.Ux * tf, n.Uy * n.Uy * tf, n.Uz * n.Uz * tf,
		n.Phi * n.Phi * tf, n.Theta * n.Theta * tf, n.Psi * n.Psi * tf,
		n.Phi0 * n.Phi0 * tf, n.Theta0 * n.Theta0 * tf, n.Psi0 * n.Psi0 * tf,
		n.Vx * n.Vx * tf, n.Vy * n.Vy * tf, n.Vz * n.Vz * tf,
	})
	s.M = *matrix.Sum(matrix.Product(&f, matrix.Product(&s.M, f.Transpose())), nn)
	s.T = c.T
}

func (s *State) Update(m Measurement, n Measurement) {
	z := s.predictMeasurement()
	y := matrix.MakeDenseMatrix([]float64{
		m.Wx - z.Wx, m.Wy - z.Wy, m.Wz - z.Wz,
		m.Sx - z.Sx, m.Sy - z.Sy, m.Sz - z.Sz,
		m.Mx - z.Mx, m.My - z.My, m.Mz - z.Mz,
	}, 9, 1)
	h := s.calcJacobianMeasurement()
	nn := matrix.Diagonal([]float64{
		n.Wx * n.Wx, n.Wy * n.Wy, n.Wz * n.Wz,
		n.Sx * n.Sx, n.Sy * n.Sy, n.Sz * n.Sz,
		n.Mx * n.Mx, n.My * n.My, n.Mz * n.Mz,
	})
	ss := *matrix.Sum(matrix.Product(&h, matrix.Product(&s.M, h.Transpose())), nn)
	m2, err := ss.Inverse()
	if err != nil {
		fmt.Printf("Can't invert %s", ss)
	}
	kk := matrix.Product(&s.M, matrix.Product(h.Transpose(), m2))
	su := matrix.Product(kk, y)
	s.Ux += su.Get(0, 0)
	s.Uy += su.Get(1, 0)
	s.Uz += su.Get(2, 0)
	s.Phi += su.Get(3, 0)
	s.Theta += su.Get(4, 0)
	s.Psi += su.Get(5, 0)
	s.Phi0 += su.Get(6, 0)
	s.Theta0 += su.Get(7, 0)
	s.Psi0 += su.Get(8, 0)
	s.Vx += su.Get(9, 0)
	s.Vy += su.Get(10, 0)
	s.Vz += su.Get(11, 0)
	s.T = m.T
	s.M = *matrix.Product(matrix.Difference(matrix.Eye(12), matrix.Product(kk, &h)), &s.M)
}

func (s *State) predictMeasurement() Measurement {
	var m Measurement
	s.doTrig()
	m.Wx = s.Vx +
		+(s.sPsi*s.cTheta)*s.Ux +
		-(s.sPsi*s.sTheta*s.sPhi+s.cPsi*s.cPhi)*s.Uy +
		-(s.sPsi*s.sTheta*s.cPhi-s.cPsi*s.sPhi)*s.Uz
	m.Wy = s.Vy +
		+(s.cPsi*s.cTheta)*s.Ux +
		-(s.cPsi*s.sTheta*s.sPhi-s.sPsi*s.cPhi)*s.Uy +
		-(s.cPsi*s.sTheta*s.cPhi+s.sPsi*s.sPhi)*s.Uz
	m.Wz = s.Vz +
		s.sTheta*s.Ux +
		s.cTheta*s.sPhi*s.Uy +
		s.cTheta*s.cPhi*s.Uz
	return m
}

func (s *State) calcJacobianState(c Control) matrix.DenseMatrix {
	dt := float64(c.T-s.T) / 1000
	data := make([][]float64, 12)
	for i := 0; i < 12; i++ {
		data[i] = make([]float64, 12)
	}
	s.doTrig()
	data[0][0] = 1                                                       // ux,ux
	data[0][4] = -G*s.cTheta*dt                                          // ux,theta
	data[0][7] = -data[0][4]                                             // ux,theta0
	data[1][1] = 1                                                       // uy,uy
	data[1][3] = -G*s.cTheta*s.cPhi*dt                                   // uy,phi
	data[1][4] = +G*s.sTheta*s.sPhi*dt                                   // uy,theta
	data[1][6] = -data[1][3]                                             // uy,phi0
	data[1][7] = -data[1][4]                                             // uy,theta0
	data[2][2] = 1                                                       // uz,uz
	data[2][3] = +G*s.cTheta*s.sPhi*dt                                   // uz,phi
	data[2][4] = +G*s.sTheta*s.cPhi*dt                                   // uz,theta
	data[2][6] = -data[2][3]                                             // uz,phi0
	data[2][7] = -data[2][4]                                             // uz,theta0
	data[3][3] = 1-(s.tTheta*s.cPhi*c.Q-s.sPhi*c.R)*dt                   // phi,phi
	data[3][4] = -(s.sPhi*c.Q+s.cPhi*c.R)/(s.cTheta*s.cTheta)*dt         // phi,theta
	data[3][6] = 1-data[3][3]                                            // phi,phi0
	data[3][7] = -data[3][4]                                             // phi,theta0
	data[4][3] = +(s.sPhi*c.Q+s.cPhi*c.R)*dt                             // theta,phi
	data[4][4] = 1                                                       // theta,theta
	data[4][6] = -data[4][3]                                             // theta,phi0
	data[5][3] = -(s.cPhi*c.Q-s.sPhi*c.R)/s.cTheta*dt                    // psi,phi
	data[5][4] = -(s.sPhi*c.Q-s.cPhi*c.R)*s.tTheta/s.cTheta*dt           // psi,theta
	data[5][5] = 1                                                       // phi0,phi0
	data[5][6] = -data[5][3]                                             // psi,phi0
	data[5][7] = -data[5][4]                                             // psi,theta0
	data[6][6] = 1                                                       // phi0,phi0
	data[7][7] = 1                                                       // theta0,theta0
	data[8][8] = 1                                                       // psi0,psi0
	data[9][9] = 1                                                       // vx,vx
	data[10][10] = 1                                                     // vy,vy
	data[11][11] = 1                                                     // vz,vz
	ff := *matrix.MakeDenseMatrixStacked(data)
	return ff
}

func (s *State) calcJacobianMeasurement() matrix.DenseMatrix {
	data := make([][]float64, 9)
	for i := 0; i < 9; i++ {
		data[i] = make([]float64, 12)
	}
	s.doTrig()
	data[0][0] = s.sPsi * s.cTheta                                           // wx,ux
	data[0][1] = -(s.sPsi*s.sTheta*s.sPhi + s.cPsi*s.cPhi)                   // wx,uy
	data[0][2] = -(s.sPsi*s.sTheta*s.cPhi - s.cPsi*s.sPhi)                   // wx,uz
	data[0][3] = -(s.sPsi*s.sTheta*s.cPhi-s.cPsi*s.sPhi)*s.Uy +
		(s.sPsi*s.sTheta*s.sPhi+s.cPsi*s.cPhi)*s.Uz                      // wx,phi
	data[0][4] = -(s.sPsi*s.sTheta*s.Ux +
		s.sPsi*s.cTheta*s.sPhi*s.Uy +
		s.sPsi*s.cTheta*s.cPhi*s.Uz)                                     // wx,theta
	data[0][5] = +(s.cPsi*s.cTheta*s.Ux -
		(s.cPsi*s.sTheta*s.sPhi-s.sPsi*s.cPhi)*s.Uy -
		(s.cPsi*s.sTheta*s.cPhi+s.sPsi*s.sPhi)*s.Uz)                     // wx,psi
	data[0][6] = -data[0][3]                                                 // wx,phi0
	data[0][7] = -data[0][4]                                                 // wx,theta0
	data[0][8] = -data[0][5]                                                 // wx,psi0
	data[0][9] = 1                                                           // wx,vx
	data[1][0] = s.cPsi * s.cTheta                                           // wy,ux
	data[1][1] = -(s.cPsi*s.sTheta*s.sPhi - s.sPsi*s.cPhi)                   // wy,uy
	data[1][2] = -(s.cPsi*s.sTheta*s.cPhi + s.sPsi*s.sPhi)                   // wy,uz
	data[1][3] = -(s.cPsi*s.sTheta*s.cPhi+s.sPsi*s.sPhi)*s.Uy +
		(s.cPsi*s.sTheta*s.sPhi-s.sPsi*s.cPhi)*s.Uz                      // wy,phi
	data[1][4] = -(s.cPsi*s.sTheta*s.Ux +
		s.cPsi*s.cTheta*s.sPhi*s.Uy +
		s.cPsi*s.cTheta*s.cPhi*s.Uz)                                     // wy,theta
	data[1][5] = -s.sPsi*s.cTheta*s.Ux +
		(s.sPsi*s.sTheta*s.sPhi+s.cPsi*s.cPhi)*s.Uy +
		(s.sPsi*s.sTheta*s.cPhi-s.cPsi*s.sPhi)*s.Uz                      // wy,psi
	data[1][6] = -data[1][3]                                                 // wy,phi0
	data[1][7] = -data[1][4]                                                 // wy,theta0
	data[1][8] = -data[1][5]                                                 // wy,psi0
	data[1][10] = 1                                                          // wy,vy
	data[2][0] = s.sTheta                                                    // wz,ux
	data[2][1] = s.cTheta * s.sPhi                                           // wz,uy
	data[2][2] = s.cTheta * s.cPhi                                           // wz,uz
	data[2][3] = s.cTheta*s.cPhi*s.Uy - s.cTheta*s.sPhi*s.Uz                 // wz,phi
	data[2][4] = s.cTheta*s.Ux - s.sTheta*s.sPhi*s.Uy - s.sTheta*s.cPhi*s.Uz // wz,theta
	data[2][6] = -data[2][3]                                                 // wz,phi0
	data[2][7] = -data[2][4]                                                 // wz,theta0
	data[2][8] = -data[2][5]                                                 // wz,psi0
	data[2][11] = 1                                                          // wz,vz
	fmt.Print("\n", data, "\n")
	hh := *matrix.MakeDenseMatrixStacked(data)
	return hh
}
