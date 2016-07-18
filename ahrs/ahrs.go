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
	tL = 2.0		// Long-term timescale for determining inertiality
	tS = 0.5		// Short-term timescale for determining inertiality
)

// State holds the complete information describing the state of the aircraft
// Order within State also defines order in the matrices below
type State struct {
	Initialized, Calibrated    bool     // Is the state valid (initialized, sensible)
	U1, U2, U3    	float64             // Vector for airspeed, aircraft (accelerated) frame
	E0, E1, E2, E3	float64             // Quaternion rotating aircraft to earth frame
	V1, V2, V3    	float64             // Vector describing windspeed, latlong axes, earth (inertial) frame
	M1, M2, M3    	float64             // Vector describing reference magnetometer direction, earth (inertial) frame
	T             	int64               // Timestamp when last updated
	M             	*matrix.DenseMatrix  // Covariance matrix of state uncertainty, same order as above vars

	tLastCal	float64		    // Last time calibration was run
	cS, cL		Control		    // Short-term and long-term control moving averages, for determining inertiality
	mS, mL	 	Measurement	    // Short-term and long-term measurement moving averages, for determining inertiality

	F0, F1, F2, F3	float64             // Calibration quaternion describing roll, pitch and heading biases due to placement of stratux, aircraft frame
	f11, f12, f13,			    // After calibration, these are quaternion fragments for rotating stratux to level
	f21, f22, f23,
	f31, f32, f33	float64
}

// Control holds the control inputs for the Kalman filter: gyro change and accelerations
type Control struct {
	H1, H2, H3 float64 // Vector of gyro rates in roll, pitch, heading axes, aircraft (accelerated) frame
	A1, A2, A3 float64 // Vector holding accelerometer readings, g's, aircraft (accelerated) frame
	T          int64   // Timestamp of readings
}

// Measurement holds the measurements used for updating the Kalman filter: groundspeed, true airspeed, magnetometer
// Note: airspeed and magnetometer may not be available until appropriate sensors are working
type Measurement struct {  // Order here also defines order in the matrices below
	WValid, UValid, MValid bool // Do we have valid GPS, airspeed and magnetometer readings?
	W1, W2, W3 float64 // Quaternion holding GPS speed in N/S, E/W and U/D directions, knots, latlong axes, earth (inertial) frame
	U1, U2, U3 float64 // Quaternion holding measured airspeed, knots, aircraft (accelerated) frame
	M1, M2, M3 float64 // Quaternion holding magnetometer readings, aircraft (accelerated) frame
	T          int64   // Timestamp of GPS, airspeed and magnetometer readings
}

// vx represents process uncertainties, per second
var vx = State{
	U1: 1, U2: 5, U3: 5,
	E0: 2e-2, E1: 2e-2, E2: 2e-2, E3: 2e-2,
	V1: 0.5, V2: 0.5, V3: 0.5,
	M1: 0.005, M2: 0.005, M3: 0.005,
	T: 1000000000,
}

// vm represents measurement uncertainties, assuming sensor is present
var vm = Measurement{
	W1: 0.2, W2: 0.2, W3: 0.2, // GPS uncertainty is small
	U1: 2, U2: 25, U3: 25, // Airspeed isn't measured yet; U2 & U3 serve to bias toward coordinated flight
	M1: 0.1, M2: 0.1, M3: 0.1, //TODO Put reasonable magnetometer values here once working
}

// normalize normalizes the E quaternion in State s
func (s *State) normalize() {
	ee := math.Sqrt(s.E0*s.E0 + s.E1*s.E1 + s.E2*s.E2 + s.E3*s.E3)
	s.E0 /= ee
	s.E1 /= ee
	s.E2 /= ee
	s.E3 /= ee
}

// IsInertial determines heuristically whether the aircraft frame is reasonably inertial
func (s *State) IsInertial(c *Control, m *Measurement) (inertial bool) {
	// Update moving averages
	var kS, kL float64
	var t0, t float64

	t0, t = s.tLastCal, float64(c.T)/1000000000
	s.tLastCal = t
	kS = tS/(tS+t-t0)
	s.cS.H1 = kS*s.cS.H1 + (1-kS)*c.H1
	s.cS.H2 = kS*s.cS.H2 + (1-kS)*c.H2
	s.cS.H3 = kS*s.cS.H3 + (1-kS)*c.H3
	s.cS.A1 = kS*s.cS.A1 + (1-kS)*c.A1
	s.cS.A2 = kS*s.cS.A2 + (1-kS)*c.A2
	s.cS.A3 = kS*s.cS.A3 + (1-kS)*c.A3
	s.cS.T  = int64((kS*float64(s.cS.T)  + (1-kS)*t*1000000000)+0.5)

	kL = tL/(tL+t-t0)
	//s.cL.H1 = kL*s.cL.H1 + (1-kL)*c.H1	// Don't need these for now
	//s.cL.H2 = kL*s.cL.H2 + (1-kL)*c.H2
	//s.cL.H3 = kL*s.cL.H3 + (1-kL)*c.H3
	s.cL.A1 = kL*s.cL.A1 + (1-kL)*c.A1
	s.cL.A2 = kL*s.cL.A2 + (1-kL)*c.A2
	s.cL.A3 = kL*s.cL.A3 + (1-kL)*c.A3
	s.cL.T  = int64((kL*float64(s.cL.T)  + (1-kL)*t*1000000000)+0.5)

	t = float64(m.T)/1000000000
	kS = tS/(tS+t-t0)
	s.mS.W1 = kS*s.mS.W1 + (1-kS)*m.W1
	s.mS.W2 = kS*s.mS.W2 + (1-kS)*m.W2
	s.mS.W3 = kS*s.mS.W3 + (1-kS)*m.W3
	s.mS.U1 = kS*s.mS.U1 + (1-kS)*m.U1
	//s.mS.M1 = kS*s.mS.M1 + (1-kS)*m.M1	// Don't need these for now
	//s.mS.M2 = kS*s.mS.M2 + (1-kS)*m.M2
	//s.mS.M3 = kS*s.mS.M3 + (1-kS)*m.M3
	s.mS.T  = int64((kS*float64(s.mS.T)  + (1-kS)*t*1000000000)+0.5)

	kL = tL/(tL+t-t0)
	s.mL.W1 = kL*s.mL.W1 + (1-kL)*m.W1
	s.mL.W2 = kL*s.mL.W2 + (1-kL)*m.W2
	s.mL.W3 = kL*s.mL.W3 + (1-kL)*m.W3
	s.mL.U1 = kL*s.mL.U1 + (1-kL)*m.U1
	//s.mL.M1 = kL*s.mL.M1 + (1-kL)*m.M1	// Don't need these for now
	//s.mL.M2 = kL*s.mL.M2 + (1-kL)*m.M2
	//s.mL.M3 = kL*s.mL.M3 + (1-kL)*m.M3
	s.mL.T  = int64((kL*float64(s.mL.T)  + (1-kL)*t*1000000000)+0.5)

	if t-float64(s.mL.T/1000000000)>tL/2 {
		// Tests for inertial frame:
		//TODO westphae: this needs to be tuned!
		// 1. Gyro rates are nearly zero
		inertial = math.Abs(s.cS.H1 - 0) < 0.1 && math.Abs(s.cS.H2 - 0) < 0.1 && math.Abs(s.cS.H3 - 0) < 0.1 &&
		// 2. Acceleration has magnitude nearly G in nearly steady direction
			math.Abs(s.cL.A1 - s.cS.A1) < 0.05 && math.Abs(s.cL.A2 - s.cS.A2) < 0.05 && math.Abs(s.cL.A3 - s.cS.A3) < 0.05 &&
		// 3. If valid, GPS speed and track are nearly steady
			(!m.WValid || (math.Abs(s.mL.W1 - s.mS.W1) < 0.1 && math.Abs(s.mL.W2 - s.mS.W2) < 0.1 && math.Abs(s.mL.W3 - s.mS.W3) < 0.05)) &&
		// 4. If valid, airspeed is nearly steady
			(!m.UValid || (math.Abs(s.mL.U1 - s.mS.U1) < 1))
		// 5. If valid, magnetometer is nearly steady
		//	(!m.MValid || (math.Abs(s.mL.M1-s.mS.M1) < 0.1 && math.Abs(s.mL.M2-s.mS.M2) < 0.05 && math.Abs(s.mL.M3-s.mS.M3) < 0.05))
		log.Printf("\nTime: %f\n", t)
		log.Printf("%f %f\n", s.mS.W2, s.mL.W2)
		log.Printf("Initialized: %t, Calibrated: %t, Inertial: %t\n", s.Initialized, s.Calibrated, inertial)
		log.Printf("Gyro:  %f %f %f\n", s.cS.H1, s.cS.H2, s.cS.H3)
		log.Printf("Accel: %f %f %f\n", s.cL.A1-s.cS.A1, s.cL.A2-s.cS.A2, s.cL.A3-s.cS.A3)
		log.Printf("GPS:   %f %f %f\n", s.mL.W1-s.mS.W1, s.mL.W2-s.mS.W2, s.mL.W3-s.mS.W3)
		log.Printf("ASI:   %f\n", s.mL.U1-s.mS.U1)
		log.Printf("Mag:   %f %f %f\n", s.mL.M1-s.mS.M1, s.mL.M2-s.mS.M2, s.mL.M3-s.mS.M3)
		log.Printf("Frame is inertial: %t\n", inertial)
	}
	return
}

// Initialize the state at the start of the Kalman filter, based on current
// measurements and controls
func (s *State) Initialize(m *Measurement, c *Control) {
	// for now just treat the case !m.UValid
	if m.WValid {
		s.U1 = math.Sqrt(m.W1 * m.W1 + m.W2 * m.W2) // Best guess at initial airspeed is initial groundspeed
	} else {
		s.U1 = 0
	}
	s.U2, s.U3 = 0, 0
	s.E1, s.E2 = 0, 0
	if m.WValid && s.U1 > 0 {
		// Best guess at initial heading is initial track
		// Simplified half-angle formulae
		s.E0, s.E3 = math.Sqrt((s.U1 + m.W1) / (2 * s.U1)), math.Sqrt((s.U1 - m.W1) / (2 * s.U1))
		if m.W2 > 0 {
			s.E3 *= -1
		}
		s.Initialized = true
	} else if !m.WValid {	// If no groundspeed available then no idea which direction we're pointing
		// assume north
		s.E0, s.E3 = math.Sqrt2/2, -math.Sqrt2/2
		s.Initialized = true
	} else {	// We're just stationary; wait until we start moving to initialize
		//TODO westphae: could use magnetometer to initially point north
		s.Initialized = false
		return
	}
	s.V1, s.V2, s.V3 = 0, 0, 0	// Best guess at initial windspeed is zero (actually headwind...)
	s.tLastCal = float64(m.T)/1000000000-3600	// If just initialized, do a fresh calibration
	s.M = matrix.Diagonal([]float64{
		20*20, 1*1, 1*1,
		0.1*0.1, 0.1*0.1, 0.1*0.1, 0.1*0.1,
		20*20, 20*20, 2*2,
		0.01*0.01, 0.01*0.01, 0.01*0.01,
	})
}

// Calibrate performs a calibration, determining the quaternion to rotate it to
// be effectively level and pointing forward.  Must be run when in an unaccelerated state.
func (s *State) Calibrate(c *Control, m *Measurement) {
	//TODO: I have the math for this, just no time to implement it yet; will do soon
	// For now, just assume it's place in the aircraft level and pointing forward
	//TODO: If m.UValid then calculate correct airspeed and windspeed;
	// Must have some change in acceleration to calibrate
	// Then match up GPS acceleration with sensor acceleration to rotate sensor into aircraft frame
	s.F0 = 1
	s.F1 = 0
	s.F2 = 0
	s.F3 = 0

	// Set the quaternion fragments to rotate from sensor frame into aircraft frame
	s.f11 = 2 * (+s.F0 * s.F0 + s.F1 * s.F1 - 0.5)
	s.f12 = 2 * (+s.F0 * s.F3 + s.F1 * s.F2)
	s.f13 = 2 * (-s.F0 * s.F2 + s.F1 * s.F3)
	s.f21 = 2 * (-s.F0 * s.F3 + s.F2 * s.F1)
	s.f22 = 2 * (+s.F0 * s.F0 + s.F2 * s.F2 - 0.5)
	s.f23 = 2 * (+s.F0 * s.F1 + s.F2 * s.F3)
	s.f31 = 2 * (+s.F0 * s.F2 + s.F3 * s.F1)
	s.f32 = 2 * (-s.F0 * s.F1 + s.F3 * s.F2)
	s.f33 = 2 * (+s.F0 * s.F0 + s.F3 * s.F3 - 0.5)

	if m.MValid {
		s.M1 = 2 * m.M1 * (s.E1 * s.E1 + s.E0 * s.E0 - 0.5) +
		2 * m.M2 * (s.E1 * s.E2 + s.E0 * s.E3) +
		2 * m.M3 * (s.E1 * s.E3 - s.E0 * s.E2)
		s.M2 = 2 * m.M1 * (s.E2 * s.E1 - s.E0 * s.E3) +
		2 * m.M2 * (s.E2 * s.E2 + s.E0 * s.E0 - 0.5) +
		2 * m.M3 * (s.E2 * s.E3 + s.E0 * s.E1)
		s.M3 = 2 * m.M1 * (s.E3 * s.E1 + s.E0 * s.E2) +
		2 * m.M2 * (s.E3 * s.E2 - s.E0 * s.E1) +
		2 * m.M3 * (s.E3 * s.E3 + s.E0 * s.E0 - 0.5)
	}

	s.Calibrated = true
}

// Predict performs the prediction phase of the Kalman filter given the control inputs
func (s *State) Predict(c *Control) {
	f := s.calcJacobianState(c)
	dt := float64(c.T-s.T) / 1000000000

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

	tf := dt / (float64(vx.T) / 1000000000)
	s.M = matrix.Sum(matrix.Product(f, matrix.Product(s.M, f.Transpose())),
		matrix.Diagonal([]float64{
			vx.U1 * vx.U1 * tf, vx.U2 * vx.U2 * tf, vx.U3 * vx.U3 * tf,
			vx.E0 * vx.E0 * tf, vx.E1 * vx.E1 * tf, vx.E2 * vx.E2 * tf, vx.E3 * vx.E3 * tf,
			vx.V1 * vx.V1 * tf, vx.V2 * vx.V2 * tf, vx.V3 * vx.V3 * tf,
			vx.M1 * vx.M1 * tf, vx.M2 * vx.M2 * tf, vx.M3 * vx.M3 * tf,
		}))
}

// Update applies the Kalman filter corrections given the measurements
func (s *State) Update(m *Measurement) {
	z := new(Measurement)
	s.PredictMeasurement(z)
	y := []float64{
		m.W1 - z.W1, m.W2 - z.W2, m.W3 - z.W3,
		m.U1 - z.U1, m.U2 - z.U2, m.U3 - z.U3,
		m.M1 - z.M1, m.M2 - z.M2, m.M3 - z.M3,
	}
	h := s.calcJacobianMeasurement()
	var mnoise = make([]float64, 9)
	if m.WValid {
		mnoise[0] = vm.W1*vm.W1
		mnoise[1] = vm.W2*vm.W2
		mnoise[2] = vm.W3*vm.W3
	} else {
		y[0] = 0
		y[1] = 0
		y[2] = 0
		mnoise[0] = 1e9
		mnoise[1] = 1e9
		mnoise[2] = 1e9
	}
	if m.UValid {
		mnoise[3] = vm.U1*vm.U1
	} else {
		y[3] = 0
		mnoise[3] = 1e9
	}
	// U2, U3 are just here to bias toward coordinated flight
	mnoise[4] = vm.U2*vm.U2
	mnoise[5] = vm.U3*vm.U3
	if m.MValid {
		mnoise[6] = vm.M1*vm.M1
		mnoise[7] = vm.M2*vm.M2
		mnoise[8] = vm.M3*vm.M3
	} else {
		y[6] = 0
		y[7] = 0
		y[8] = 0
		mnoise[6] = 1e9
		mnoise[7] = 1e9
		mnoise[8] = 1e9
	}
	ss := *matrix.Sum(matrix.Product(h, matrix.Product(s.M, h.Transpose())), matrix.Diagonal(mnoise))

	m2, err := ss.Inverse()
	if err != nil {
		log.Println("AHRS: Can't invert Kalman gain matrix")
	}
	kk := matrix.Product(s.M, matrix.Product(h.Transpose(), m2))
	su := matrix.Product(kk, matrix.MakeDenseMatrix(y, 9, 1))
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
	s.M = matrix.Product(matrix.Difference(matrix.Eye(13), matrix.Product(kk, h)), s.M)
}

func (s *State) PredictMeasurement(m *Measurement) {
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

	m.M1 =  2*s.M1*(s.E1*s.E1+s.E0*s.E0-0.5) +
		2*s.M2*(s.E1*s.E2-s.E0*s.E3) +
		2*s.M3*(s.E1*s.E3+s.E0*s.E2)
	m.M2 =  2*s.M1*(s.E2*s.E1+s.E0*s.E3) +
		2*s.M2*(s.E2*s.E2+s.E0*s.E0-0.5) +
		2*s.M3*(s.E2*s.E3-s.E0*s.E1)
	m.M3 =  2*s.M1*(s.E3*s.E1-s.E0*s.E2) +
		2*s.M2*(s.E3*s.E2+s.E0*s.E1) +
		2*s.M3*(s.E3*s.E3+s.E0*s.E0-0.5)
}

func (s *State) calcJacobianState(c *Control) *matrix.DenseMatrix {
	dt := float64(c.T-s.T) / 1000000000
	data := make([][]float64, 13)
	for i := 0; i < 13; i++ {
		data[i] = make([]float64, 13)
	}

	// Apply the calibration quaternion F to rotate the stratux sensors to level
	h1 := c.H1*s.f11 + c.H2*s.f12 + c.H3*s.f13
	h2 := c.H1*s.f21 + c.H2*s.f22 + c.H3*s.f23
	h3 := c.H1*s.f31 + c.H2*s.f32 + c.H3*s.f33

	data[0][0] = 1                                         // U1,U1
	data[0][1] = -h3*dt  // U1,U2
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
	data[2][3] = -2*G*s.E0*dt   // U3/E0
	data[2][4] = +2*G*s.E1*dt              // U3/E1
	data[2][5] = +2*G*s.E2*dt              // U3/E2
	data[2][6] = -2*G*s.E3*dt   // U3/E3
	data[3][3] = 1                                         // E0/E0
	data[3][4] = -0.5*dt*h1	// E0/E1
	data[3][5] = -0.5*dt*h2	// E0/E2
	data[3][6] = -0.5*dt*h3	// E0/E3
	data[4][3] = +0.5*dt*h1	// E1/E0
	data[4][4] = 1                                         // E1/E1
	data[4][5] = -0.5*dt*h3	// E1/E2
	data[4][6] = +0.5*dt*h2	// E1/E3
	data[5][3] = +0.5*dt*h2	// E2/E0
	data[5][4] = +0.5*dt*h3	// E2/E1
	data[5][5] = 1                                         // E2/E2
	data[5][6] = -0.5*dt*h1	// E2/E3
	data[6][3] = +0.5*dt*h3	// E3/E0
	data[6][4] = -0.5*dt*h2	// E3/E1
	data[6][5] = +0.5*dt*h1	// E3/E2
	data[6][6] = 1                                         // E3/E3
	data[7][7] = 1                                         // V1/V1
	data[8][8] = 1                                         // V2/V2
	data[9][9] = 1                                         // V3/V3
	data[10][10] = 1                                       // M1/M1
	data[11][11] = 1                                       // M2/M2
	data[12][12] = 1                                       // M3/M3

	return matrix.MakeDenseMatrixStacked(data)
}

func (s *State) calcJacobianMeasurement() *matrix.DenseMatrix {
	data := make([][]float64, 9)
	for i := 0; i < 9; i++ {
		data[i] = make([]float64, 13)
	}

	data[0][0] = 2*(s.E1*s.E1+s.E0*s.E0-0.5)                    // W1/U1
	data[0][1] = 2*(s.E1*s.E2+s.E0*s.E3)                        // W1/U2
	data[0][2] = 2*(s.E1*s.E3-s.E0*s.E2)                        // W1/U3
	data[0][3] = 2*(+s.E0*s.U1 + s.E3*s.U2 - s.E2*s.U3)         // W1/E0
	data[0][4] = 2*(+s.E1*s.U1 + s.E2*s.U2 + s.E3*s.U3)         // W1/E1
	data[0][5] = 2*(-s.E2*s.U1 + s.E1*s.U2 - s.E0*s.U3)         // W1/E2
	data[0][6] = 2*(-s.E3*s.U1 + s.E0*s.U2 + s.E1*s.U3)         // W1/E3
	data[0][7] = 1                                              // W1/V1
	data[1][0] = 2*(s.E2*s.E1 - s.E0*s.E3)                      // W2/U1
	data[1][1] = 2*(s.E2*s.E2 + s.E0*s.E0 - 0.5)		    // W2/U2
	data[1][2] = 2*(s.E2*s.E3 + s.E0*s.E1)                      // W2/U3
	data[1][3] = 2*(-s.E3*s.U1 + s.E0*s.U2 + s.E1*s.U3)         // W2/E0
	data[1][4] = 2*(+s.E2*s.U1 - s.E1*s.U2 + s.E0*s.U3)         // W2/E1
	data[1][5] = 2*(+s.E1*s.U1 + s.E2*s.U2 + s.E3*s.U3)         // W2/E2
	data[1][6] = 2*(-s.E0*s.U1 - s.E3*s.U2 + s.E2*s.U3)         // W2/E3
	data[1][8] = 1                                              // W2/V2
	data[2][0] = 2*(s.E3*s.E1 + s.E0*s.E2)                      // W3/U1
	data[2][1] = 2*(s.E3*s.E2 - s.E0*s.E1)                      // W3/U2
	data[2][2] = 2*(s.E3*s.E3 + s.E0*s.E0 - 0.5)                // W3/U3
	data[2][3] = 2*(+s.E2*s.U1 - s.E1*s.U2 + s.E0*s.U3)         // W3/E0
	data[2][4] = 2*(+s.E3*s.U1 - s.E0*s.U2 - s.E1*s.U3)         // W3/E1
	data[2][5] = 2*(+s.E0*s.U1 + s.E3*s.U2 - s.E2*s.U3)         // W3/E2
	data[2][6] = 2*(+s.E1*s.U1 + s.E2*s.U2 + s.E3*s.U3)         // W3/E3
	data[2][9] = 1                                              // W3/V3

	data[3][0] = 1                                              // U1/U1
	data[4][1] = 1                                              // U2/U2
	data[5][2] = 1                                              // U3/U3

	data[6][3] = 2*(+s.E0*s.M1 - s.E3*s.M2 + s.E2*s.M3)         // M1/E0
	data[6][4] = 2*(+s.E1*s.M1 + s.E2*s.M2 + s.E3*s.M3)         // M1/E1
	data[6][5] = 2*(-s.E2*s.M1 + s.E1*s.M2 + s.E0*s.M3)         // M1/E2
	data[6][6] = 2*(-s.E3*s.M1 - s.E0*s.M2 + s.E1*s.M3)         // M1/E3
	data[6][10] = 2*(s.E1*s.E1+s.E0*s.E0-0.5)                   // M1/M1
	data[6][11] = 2*(s.E1*s.E2-s.E0*s.E3)                       // M1/M2
	data[6][12] = 2*(s.E1*s.E3+s.E0*s.E2)                       // M1/M3
	data[7][3] =  2*(+s.E3*s.M1 + s.E0*s.M2 - s.E1*s.M3)        // M2/E0
	data[7][4] =  2*(+s.E2*s.M1 - s.E1*s.M2 - s.E0*s.M3)        // M2/E1
	data[7][5] =  2*(+s.E1*s.M1 + s.E2*s.M2 + s.E3*s.M3)        // M2/E2
	data[7][6] =  2*(+s.E0*s.M1 - s.E3*s.M2 + s.E2*s.M3)        // M2/E3
	data[7][10] = 2*(s.E2*s.E1 + s.E0*s.E3)                     // M2/M1
	data[7][11] = 2*(s.E2*s.E2 + s.E0*s.E0 - 0.5)               // M2/M2
	data[7][12] = 2*(s.E2*s.E3 - s.E0*s.E1)                     // M2/M3
	data[8][3] =  2*(-s.E2*s.M1 + s.E1*s.M2 + s.E0*s.M3)        // M3/E0
	data[8][4] =  2*(+s.E3*s.M1 + s.E0*s.M2 - s.E1*s.M3)        // M3/E1
	data[8][5] =  2*(-s.E0*s.M1 + s.E3*s.M2 - s.E2*s.M3)        // M3/E2
	data[8][6] =  2*(+s.E1*s.M1 + s.E2*s.M2 + s.E3*s.M3)        // M3/E3
	data[8][10] = 2*(s.E3*s.E1 - s.E0*s.E2)                     // M3/M1
	data[8][11] = 2*(s.E3*s.E2 + s.E0*s.E1)                     // M3/M2
	data[8][12] = 2*(s.E3*s.E3 + s.E0*s.E0 - 0.5)               // M3/M3

	return matrix.MakeDenseMatrixStacked(data)
}
