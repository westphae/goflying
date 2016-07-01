// Test out the AHRS code in ahrs/ahrs.go.
// Define a flight path/attitude in code, and then synthesize the matching GPS, gyro, accel (and other) data
// Add some noise if desired.
// Then see if the AHRS code can replicate the "true" attitude given the noisy and limited input data
package main

import (
	"errors"
	"fmt"
	"math"
	"net/http"
	"os"
	"sort"
	"sync"

	"github.com/skelterjohn/go.matrix"
	"github.com/westphae/goflying/ahrs"
)

const (
	dt  = 0.1
	deg = 180 / math.Pi
)

// Situation defines a scenario by piecewise-linear interpolation
type Situation struct {
	t                  []float64 // times for situation, s
	u1, u2, u3         []float64 // airspeed, kts, aircraft frame [F/B, R/L, and U/D]
	phi, theta, psi    []float64 // attitude, rad [roll, pitch, heading]
	phi0, theta0, psi0 []float64 // base attitude, rad [adjust for position of stratu1 on glareshield]
	v1, v2, v3         []float64 // windspeed, kts, earth frame [N/S, E/W, and U/D]
	m1, m2, m3         []float64 // magnetometer reading
	init               sync.Once // only convert from euler to quaternions once
	e0, e1, e2, e3     []float64 // rotation quaternion, calculated based on phi/theta/psi
	f0, f1, f2, f3     []float64 // calibration quaternion, calculated based on phi/theta/psi
}

// ToQuaternion calculates the 0,1,2,3 components of the rotation quaternion
// corresponding to the Tait-Bryan angles phi, theta, psi
func ToQuaternion(phi, theta, psi float64) (float64, float64, float64, float64) {
	cphi := math.Cos(-phi / 2)
	sphi := math.Sin(-phi / 2)
	ctheta := math.Cos(-theta / 2)
	stheta := math.Sin(-theta / 2)
	cpsi := math.Cos(-psi / 2)
	spsi := math.Sin(-psi / 2)

	q0 := cphi*ctheta*cpsi + sphi*stheta*spsi
	q1 := sphi*ctheta*cpsi - cphi*stheta*spsi
	q2 := cphi*stheta*cpsi + sphi*ctheta*spsi
	q3 := cphi*ctheta*spsi - sphi*stheta*cpsi
	return q0, q1, q2, q3
}

// FromQuaternion calculates the Tait-Bryan angles phi, theta, phi corresponding to
// the quaternion
func FromQuaternion(q0, q1, q2, q3 float64) (float64, float64, float64) {
	phi := math.Atan2(2*(q0*q1+q2*q3), 2*(q1*q1+q2*q2)-1)
	theta := math.Asin(2 * (q3*q1 - q0*q2))
	psi := math.Atan2(2*(q0*q3+q1*q2), 2*(q2*q2+q3*q3)-1)
	return phi, theta, psi
}

// Interpolate an ahrs.State from a Situation definition at a given time
func (s *Situation) interpolate(t float64) (ahrs.State, error) {
	if t < s.t[0] || t > s.t[len(s.t)-1] {
		return ahrs.State{}, errors.New("requested time is outside of scenario")
	}
	ix := 0
	if t > s.t[0] {
		ix = sort.SearchFloat64s(s.t, t) - 1
	}

	s.init.Do(func() {
		fmt.Println("interpolate running init")
		s.e0 = make([]float64, len(s.t))
		s.e1 = make([]float64, len(s.t))
		s.e2 = make([]float64, len(s.t))
		s.e3 = make([]float64, len(s.t))
		s.f0 = make([]float64, len(s.t))
		s.f1 = make([]float64, len(s.t))
		s.f2 = make([]float64, len(s.t))
		s.f3 = make([]float64, len(s.t))
		for i := 0; i < len(s.t); i++ {
			s.e0[i], s.e1[i], s.e2[i], s.e3[i] = ToQuaternion(s.phi[i], s.theta[i], s.phi[i])
			s.f0[i], s.f1[i], s.f2[i], s.f3[i] = ToQuaternion(s.phi0[i], s.theta0[i], s.phi0[i])
		}
	})

	f := (s.t[ix+1] - t) / (s.t[ix+1] - s.t[ix])
	return ahrs.State{
		U1: f*s.u1[ix] + (1-f)*s.u1[ix+1],
		U2: f*s.u2[ix] + (1-f)*s.u2[ix+1],
		U3: f*s.u3[ix] + (1-f)*s.u3[ix+1],
		E0: f*s.e0[ix] + (1-f)*s.e0[ix+1],
		E1: f*s.e1[ix] + (1-f)*s.e1[ix+1],
		E2: f*s.e2[ix] + (1-f)*s.e2[ix+1],
		E3: f*s.e3[ix] + (1-f)*s.e3[ix+1],
		F0: f*s.f0[ix] + (1-f)*s.f0[ix+1],
		F1: f*s.f1[ix] + (1-f)*s.f1[ix+1],
		F2: f*s.f2[ix] + (1-f)*s.f2[ix+1],
		F3: f*s.f3[ix] + (1-f)*s.f3[ix+1],
		V1: f*s.v1[ix] + (1-f)*s.v1[ix+1],
		V2: f*s.v2[ix] + (1-f)*s.v2[ix+1],
		V3: f*s.v3[ix] + (1-f)*s.v3[ix+1],
		M1: f*s.m1[ix] + (1-f)*s.m1[ix+1],
		M2: f*s.m2[ix] + (1-f)*s.m2[ix+1],
		M3: f*s.m3[ix] + (1-f)*s.m3[ix+1],
		T:  uint32(t*1000 + 0.5),
		M:  matrix.DenseMatrix{},
	}, nil
}

// Determine time derivative of an ahrs.State from a Situation definition at a given time
func (s *Situation) derivative(t float64) (ahrs.State, error) {
	if t < s.t[0] || t > s.t[len(s.t)-1] {
		return ahrs.State{}, errors.New("requested time is outside of scenario")
	}
	ix := 0
	if t > s.t[0] {
		ix = sort.SearchFloat64s(s.t, t) - 1
	}

	s.init.Do(func() {
		fmt.Println("derivative running init")
		s.e0 = make([]float64, len(s.t))
		s.e1 = make([]float64, len(s.t))
		s.e2 = make([]float64, len(s.t))
		s.e3 = make([]float64, len(s.t))
		s.f0 = make([]float64, len(s.t))
		s.f1 = make([]float64, len(s.t))
		s.f2 = make([]float64, len(s.t))
		s.f3 = make([]float64, len(s.t))
		for i := 0; i < len(s.e0); i++ {
			s.e0[i], s.e1[i], s.e2[i], s.e3[i] = ToQuaternion(s.phi[i], s.theta[i], s.phi[i])
			s.f0[i], s.f1[i], s.f2[i], s.f3[i] = ToQuaternion(s.phi0[i], s.theta0[i], s.phi0[i])
		}
	})

	dt := s.t[ix+1] - s.t[ix]
	return ahrs.State{
		U1: (s.u1[ix+1] - s.u1[ix]) / dt,
		U2: (s.u2[ix+1] - s.u2[ix]) / dt,
		U3: (s.u3[ix+1] - s.u3[ix]) / dt,
		E0: (s.e0[ix+1] - s.e0[ix]) / dt,
		E1: (s.e1[ix+1] - s.e1[ix]) / dt,
		E2: (s.e2[ix+1] - s.e2[ix]) / dt,
		E3: (s.e3[ix+1] - s.e3[ix]) / dt,
		F0: (s.f0[ix+1] - s.f0[ix]) / dt,
		F1: (s.f1[ix+1] - s.f1[ix]) / dt,
		F2: (s.f2[ix+1] - s.f2[ix]) / dt,
		F3: (s.f3[ix+1] - s.f3[ix]) / dt,
		V1: (s.v1[ix+1] - s.v1[ix]) / dt,
		V2: (s.v2[ix+1] - s.v2[ix]) / dt,
		V3: (s.v3[ix+1] - s.v3[ix]) / dt,
		M1: (s.m1[ix+1] - s.m1[ix]) / dt,
		M2: (s.m2[ix+1] - s.m2[ix]) / dt,
		M3: (s.m3[ix+1] - s.m3[ix]) / dt,
		T:  uint32(t*1000 + 0.5), // easy rounding for uint
		M:  matrix.DenseMatrix{},
	}, nil
}

// Determine ahrs.Control variables from a Situation definition at a given time
func (s *Situation) control(t float64) (ahrs.Control, error) {
	x, erri := s.interpolate(t)
	dx, errd := s.derivative(t)
	if erri != nil || errd != nil {
		return ahrs.Control{}, errors.New("requested time is outside of scenario")
	}

	// f fragments to reverse-rotate by f (airplane frame to sensor frame)
	f11 := 2 * (+x.F0*x.F0 + x.F1*x.F1 - 0.5)
	f12 := 2 * (+x.F0*x.F3 + x.F1*x.F2)
	f13 := 2 * (-x.F0*x.F2 + x.F1*x.F3)
	f21 := 2 * (-x.F0*x.F3 + x.F2*x.F1)
	f22 := 2 * (+x.F0*x.F0 + x.F2*x.F2 - 0.5)
	f23 := 2 * (+x.F0*x.F1 + x.F2*x.F3)
	f31 := 2 * (+x.F0*x.F2 + x.F3*x.F1)
	f32 := 2 * (-x.F0*x.F1 + x.F3*x.F2)
	f33 := 2 * (+x.F0*x.F0 + x.F3*x.F3 - 0.5)

	y1 := -2*(x.E0*x.E2-x.E1*x.E3) + (-dx.U1+
		+2*x.U2*(x.E0*dx.E3-x.E1*dx.E2+x.E2*dx.E1-x.E3*dx.E0)+
		-2*x.U3*(x.E0*dx.E2+x.E1*dx.E3-x.E2*dx.E0-x.E3*dx.E1))/ahrs.G
	y2 := +2*(x.E0*x.E1+x.E2*x.E3) + (-2*x.U1*(x.E0*dx.E3-x.E1*dx.E2+x.E2*dx.E1-x.E3*dx.E0)+
		-dx.U2+
		+2*x.U3*(x.E0*dx.E1-x.E1*dx.E0-x.E2*dx.E3+x.E3*dx.E2))/ahrs.G
	y3 := +2*(x.E0*x.E0+x.E3*x.E3-0.5) + (+2*x.U1*(x.E0*dx.E2+x.E1*dx.E3-x.E2*dx.E0-x.E3*dx.E1)+
		-2*x.U2*(x.E0*dx.E1-x.E1*dx.E0-x.E2*dx.E3+x.E3*dx.E2)+
		-dx.U3)/ahrs.G

	c := ahrs.Control{
		H0: dx.E0,
		H1: dx.E0 + dx.E1*f11 + dx.E2*f12 + dx.E3*f13,
		H2: dx.E0 + dx.E1*f21 + dx.E2*f22 + dx.E3*f23,
		H3: dx.E0 + dx.E1*f31 + dx.E2*f32 + dx.E3*f33,
		A1: y1*f11 + y2*f12 + y3*f13,
		A2: y1*f21 + y2*f22 + y3*f23,
		A3: y1*f31 + y2*f32 + y3*f33,
		T:  uint32(t*1000 + 0.5),
	}
	return c, nil
}

// Determine ahrs.Measurement variables from a Situation definition at a given time
func (s *Situation) measurement(t float64) (ahrs.Measurement, error) {
	if t < s.t[0] || t > s.t[len(s.t)-1] {
		return ahrs.Measurement{}, errors.New("requested time is outside of scenario")
	}
	x, _ := s.interpolate(t)

	e11 := 2 * (+x.E0*x.E0 + x.E1*x.E1 - 0.5)
	e12 := 2 * (-x.E0*x.E3 + x.E1*x.E2)
	e13 := 2 * (+x.E0*x.E2 + x.E1*x.E3)
	e21 := 2 * (+x.E0*x.E3 + x.E2*x.E1)
	e22 := 2 * (+x.E0*x.E0 + x.E2*x.E2 - 0.5)
	e23 := 2 * (-x.E0*x.E1 + x.E2*x.E3)
	e31 := 2 * (-x.E0*x.E2 + x.E3*x.E1)
	e32 := 2 * (+x.E0*x.E1 + x.E3*x.E2)
	e33 := 2 * (+x.E0*x.E0 + x.E3*x.E3 - 0.5)

	m := ahrs.Measurement{
		W1: x.U1*e11 + x.U2*e12 + x.U3*e13 + x.V1,
		W2: x.U1*e21 + x.U2*e22 + x.U3*e23 + x.V2,
		W3: x.U1*e31 + x.U2*e32 + x.U3*e33 + x.V3,
		M1: x.M1*e11 + x.M2*e12 + x.M3*e13,
		M2: x.M1*e21 + x.M2*e22 + x.M3*e23,
		M3: x.M1*e31 + x.M2*e32 + x.M3*e33,
		U1: x.U1,
		U2: x.U2,
		U3: x.U3,
		T:  uint32(t*1000 + 0.5),
	}
	return m, nil
}

// Data to define a piecewise-linear turn, with entry and exit
var airspeed = 120.0                                            // Nice airspeed for maneuvers, kts
var bank = math.Atan((2 * math.Pi * airspeed) / (ahrs.G * 120)) // Bank angle for std rate turn at given airspeed
var sitTurnDef = Situation{                                     // start, initiate roll-in, end roll-in, initiate roll-out, end roll-out, end
	t:      []float64{0, 10, 15, 135, 140, 150},
	u1:     []float64{airspeed, airspeed, airspeed, airspeed, airspeed, airspeed},
	u2:     []float64{0, 0, 0, 0, 0, 0},
	u3:     []float64{0, 0, 0, 0, 0, 0},
	phi:    []float64{0, 0, bank, bank, 0, 0},
	theta:  []float64{0, 0, math.Pi / 90, math.Pi / 90, 0, 0},
	psi:    []float64{0, 0, 0, 2 * math.Pi, 2 * math.Pi, 2 * math.Pi},
	phi0:   []float64{0, 0, 0, 0, 0, 0},
	theta0: []float64{0, 0, 0, 0, 0, 0},
	psi0:   []float64{0, 0, 0, 0, 0, 0},
	v1:     []float64{3, 3, 3, 3, 3, 3},
	v2:     []float64{4, 4, 4, 4, 4, 4},
	v3:     []float64{0, 0, 0, 0, 0, 0},
	m1:     []float64{0, 0, 0, 0, 0, 0},
	m2:     []float64{0, 0, 0, 0, 0, 0},
	m3:     []float64{0, 0, 0, 0, 0, 0},
}

func main() {
	// Files to save data to for analysis
	fActual, err := os.Create("k_state.csv")
	if err != nil {
		panic(err)
	}
	defer fActual.Close()
	fmt.Fprint(fActual, "T,Ux,Uy,Uz,Phi,Theta,Psi,Phi0,Theta0,Psi0,Vx,Vy,Vz\n")
	fKalman, err := os.Create("k_kalman.csv")
	if err != nil {
		panic(err)
	}
	defer fKalman.Close()
	fmt.Fprint(fKalman, "T,Ux,Uy,Uz,Phi,Theta,Psi,Phi0,Theta0,Psi0,Vx,Vy,Vz\n")
	fPredict, err := os.Create("k_predict.csv")
	if err != nil {
		panic(err)
	}
	defer fPredict.Close()
	fmt.Fprint(fPredict, "T,Ux,Uy,Uz,Phi,Theta,Psi,Phi0,Theta0,Psi0,Vx,Vy,Vz\n")
	fVar, err := os.Create("k_var.csv")
	if err != nil {
		panic(err)
	}
	defer fVar.Close()
	fmt.Fprint(fVar, "T,Ux,Uy,Uz,Phi,Theta,Psi,Phi0,Theta0,Psi0,Vx,Vy,Vz\n")
	fControl, err := os.Create("k_control.csv")
	if err != nil {
		panic(err)
	}
	defer fControl.Close()
	fmt.Fprint(fControl, "T,P,Q,R,Ax,Ay,Az\n")
	fMeas, err := os.Create("k_meas.csv")
	if err != nil {
		panic(err)
	}
	defer fMeas.Close()
	fmt.Fprint(fMeas, "T,Wx,Wy,Wz\n")

	s := ahrs.X0 // Initialize Kalman with a sensible starting state
	fmt.Println("Running Simulation")
	for t := sitTurnDef.t[0]; t < sitTurnDef.t[len(sitTurnDef.t)-1]; t += dt {
		s0, err := sitTurnDef.interpolate(t)
		if err != nil {
			fmt.Printf("Error interpolating at time %f: %s", t, err.Error())
			panic(err)
		}
		phi, theta, psi := FromQuaternion(s0.E0, s0.E1, s0.E2, s0.E3)
		phi0, theta0, psi0 := FromQuaternion(s0.F0, s0.F1, s0.F2, s0.F3)
		fmt.Fprintf(fActual, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
			float64(s0.T)/1000, s0.U1, s0.U2, s0.U3, phi, theta, psi,
			phi0, theta0, psi0, s0.V1, s0.V2, s0.V3)

		c, err := sitTurnDef.control(t)
		if err != nil {
			fmt.Printf("Error calculating control value at time %f: %s", t, err.Error())
			panic(err)
		}
		P, Q, R := FromQuaternion(c.H0, c.H1, c.H2, c.H3)
		fmt.Fprintf(fControl, "%f,%f,%f,%f,%f,%f,%f\n",
			float64(c.T)/1000, P, Q, R, c.A1, c.A2, c.A3)
		s.Predict(c, ahrs.VX)
		phi, theta, psi = FromQuaternion(s.E0, s.E1, s.E2, s.E3)
		phi0, theta0, psi0 = FromQuaternion(s.F0, s.F1, s.F2, s.F3)
		fmt.Fprintf(fPredict, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
			float64(s.T)/1000, s.U1, s.U2, s.U3, phi, theta, psi,
			phi0, theta0, psi0, s.V1, s.V2, s.V3)

		m, err := sitTurnDef.measurement(t)
		if err != nil {
			fmt.Printf("Error calculating measurement value at time %f: %s", t, err.Error())
			panic(err)
		}
		fmt.Fprintf(fMeas, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
			float64(m.T)/1000, m.W1, m.W2, m.W3, m.M1, m.M2, m.M3, m.U1, m.U2, m.U3)
		s.Update(m, ahrs.VM)
		phi, theta, psi = FromQuaternion(s.E0, s.E1, s.E2, s.E3)
		phi0, theta0, psi0 = FromQuaternion(s.F0, s.F1, s.F2, s.F3)
		fmt.Fprintf(fKalman, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
			float64(s.T)/1000, s.U1, s.U2, s.U3, phi, theta, psi,
			phi0, theta0, psi0, s.V1, s.V2, s.V3)
		fmt.Fprintf(fVar, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
			float64(s.T)/1000, math.Sqrt(s.M.Get(0, 0)), math.Sqrt(s.M.Get(1, 1)), math.Sqrt(s.M.Get(2, 2)),
			math.Sqrt(s.M.Get(3, 3)), math.Sqrt(s.M.Get(4, 4)), math.Sqrt(s.M.Get(5, 5)),
			math.Sqrt(s.M.Get(6, 6)), math.Sqrt(s.M.Get(7, 7)), math.Sqrt(s.M.Get(8, 8)),
			math.Sqrt(s.M.Get(9, 9)), math.Sqrt(s.M.Get(10, 10)), math.Sqrt(s.M.Get(11, 11)),
		)

		/*
			fmt.Printf("%6.2f,   %6.1f, %6.1f, %6.1f,  %7.1f, %7.1f, %7.1f,  %7.1f, %7.1f, %7.1f,  %6.1f, %6.1f, %6.1f\n",
				float64(s.T)/1000, s.Ux-s0.Ux, s.Uy-s0.Uy, s.Uz-s0.Uz,
				math.Mod((s.Phi-s0.Phi)*DEG, 360), math.Mod((s.Theta-s0.Theta)*DEG, 360), math.Mod((s.Psi-s0.Psi)*DEG, 360),
				math.Mod((s.Phi0-s0.Phi0)*DEG, 360), math.Mod((s.Theta0-s0.Theta0)*DEG, 360), math.Mod((s.Psi0-s0.Psi0)*DEG, 360),
				s.Vx-s0.Vx, s.Vy-s0.Vy, s.Vz-s0.Vz)
		*/
	}
	/*
		fmt.Print("\nFinal Uncertainty:\n")
		fmt.Printf("%6.2f,   %6.1f, %6.1f, %6.1f,  %7.1f, %7.1f, %7.1f,  %7.1f, %7.1f, %7.1f,  %6.1f, %6.1f, %6.1f\n",
			float64(s.T)/1000, math.Sqrt(s.M.Get(0,0)), math.Sqrt(s.M.Get(1,1)),math.Sqrt(s.M.Get(2,2)),
			math.Sqrt(s.M.Get(3,3))*DEG,math.Sqrt(s.M.Get(4,4))*DEG,math.Sqrt(s.M.Get(5,5))*DEG,
			math.Sqrt(s.M.Get(6,6))*DEG,math.Sqrt(s.M.Get(7,7))*DEG,math.Sqrt(s.M.Get(8,8))*DEG,
			math.Sqrt(s.M.Get(9,9)), math.Sqrt(s.M.Get(10,10)),math.Sqrt(s.M.Get(11,11)),
		)
	*/

	fmt.Println("Serving charts")
	http.Handle("/", http.FileServer(http.Dir("./")))
	http.ListenAndServe(":8080", nil)
}
