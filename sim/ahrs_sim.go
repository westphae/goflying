// Test out the AHRS code in ahrs/ahrs.go.
// Define a flight path/attitude in code, and then synthesize the matching GPS, gyro, accel (and other) data
// Add some noise if desired.
// Then see if the AHRS code can replicate the "true" attitude given the noisy and limited input data
package main

import (
	"os"
	"errors"
	"net/http"
	"fmt"
	"github.com/skelterjohn/go.matrix"
	"math"
	"sort"
	"github.com/westphae/stratux/ahrs"
)


const (
	DT = 0.1
	DEG = 180/math.Pi
)


type Situation struct {
	t                  []float64 // times for situation, s
	ux, uy, uz         []float64 // airspeed, kts, aircraft frame [F/B, R/L, and U/D]
	phi, theta, psi    []float64 // attitude, rad [roll, pitch, heading]
	phi0, theta0, psi0 []float64 // base attitude, rad [adjust for position of stratux on glareshield]
	vx, vy, vz         []float64 // windspeed, kts, earth frame [N/S, E/W, and U/D]
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
	f := (s.t[ix+1] - t) / (s.t[ix+1] - s.t[ix])
	return ahrs.State{
		Ux:     f*s.ux[ix] + (1-f)*s.ux[ix+1],
		Uy:     f*s.uy[ix] + (1-f)*s.uy[ix+1],
		Uz:     f*s.uz[ix] + (1-f)*s.uz[ix+1],
		Phi:    f*s.phi[ix] + (1-f)*s.phi[ix+1],
		Theta:  f*s.theta[ix] + (1-f)*s.theta[ix+1],
		Psi:    f*s.psi[ix] + (1-f)*s.psi[ix+1],
		Phi0:   f*s.phi0[ix] + (1-f)*s.phi0[ix+1],
		Theta0: f*s.theta0[ix] + (1-f)*s.theta0[ix+1],
		Psi0:   f*s.psi0[ix] + (1-f)*s.psi0[ix+1],
		Vx:     f*s.vx[ix] + (1-f)*s.vx[ix+1],
		Vy:     f*s.vy[ix] + (1-f)*s.vy[ix+1],
		Vz:     f*s.vz[ix] + (1-f)*s.vz[ix+1],
		T:      uint32(t * 1000 + 0.5),
		M:      matrix.DenseMatrix{},
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
	dt := s.t[ix+1] - s.t[ix]
	return ahrs.State{
		Ux:     (s.ux[ix+1] - s.ux[ix]) / dt,
		Uy:     (s.uy[ix+1] - s.uy[ix]) / dt,
		Uz:     (s.uz[ix+1] - s.uz[ix]) / dt,
		Phi:    (s.phi[ix+1] - s.phi[ix]) / dt,
		Theta:  (s.theta[ix+1] - s.theta[ix]) / dt,
		Psi:    (s.psi[ix+1] - s.psi[ix]) / dt,
		Phi0:   (s.phi0[ix+1] - s.phi0[ix]) / dt,
		Theta0: (s.theta0[ix+1] - s.theta0[ix]) / dt,
		Psi0:   (s.psi0[ix+1] - s.psi0[ix]) / dt,
		Vx:     (s.vx[ix+1] - s.vx[ix]) / dt,
		Vy:     (s.vy[ix+1] - s.vy[ix]) / dt,
		Vz:     (s.vz[ix+1] - s.vz[ix]) / dt,
		T:      uint32(t * 1000 + 0.5),
		M:      matrix.DenseMatrix{},
	}, nil
}

// Determine ahrs.Control variables from a Situation definition at a given time
func (s *Situation) control(t float64) (ahrs.Control, error) {
	if t < s.t[0] || t > s.t[len(s.t)-1] {
		return ahrs.Control{}, errors.New("requested time is outside of scenario")
	}
	x, _ := s.interpolate(t)
	dx, _ := s.derivative(t)
	c := ahrs.Control{
		P: (dx.Phi - math.Sin(x.Theta-x.Theta0)*dx.Psi),
		Q: (-math.Cos(x.Phi-x.Phi0)*dx.Theta - math.Cos(x.Theta-x.Theta0)*math.Sin(x.Phi-x.Phi0)*dx.Psi),
		R: (math.Sin(x.Phi-x.Phi0)*dx.Theta - math.Cos(x.Theta-x.Theta0)*math.Cos(x.Phi-x.Phi0)*dx.Psi),
		T: uint32(t * 1000 + 0.5),
	}
	c.Ax = (-dx.Ux+c.R*x.Uy-c.Q*x.Uz)/ahrs.G - math.Sin(x.Theta-x.Theta0)
	c.Ay = (-dx.Uy+c.P*x.Uz-c.R*x.Ux)/ahrs.G - math.Cos(x.Theta-x.Theta0)*math.Sin(x.Phi-x.Phi0)
	c.Az = (-dx.Uz+c.Q*x.Ux-c.P*x.Uy)/ahrs.G - math.Cos(x.Theta-x.Theta0)*math.Cos(x.Phi-x.Phi0)
	return c, nil
}

// Determine ahrs.Measurement variables from a Situation definition at a given time
func (s *Situation) measurement(t float64) (ahrs.Measurement, error) {
	if t < s.t[0] || t > s.t[len(s.t)-1] {
		return ahrs.Measurement{}, errors.New("requested time is outside of scenario")
	}
	x, _ := s.interpolate(t)
	m := ahrs.Measurement{
		Wx: x.Vx + math.Sin(x.Psi-x.Psi0)*math.Cos(x.Theta-x.Theta0)*x.Ux +
			-(math.Sin(x.Psi-x.Psi0)*math.Sin(x.Theta-x.Theta0)*math.Sin(x.Phi-x.Phi0)+
				math.Cos(x.Psi-x.Psi0)*math.Cos(x.Phi-x.Phi0))*x.Uy +
			-(math.Sin(x.Psi-x.Psi0)*math.Sin(x.Theta-x.Theta0)*math.Cos(x.Phi-x.Phi0)-
				math.Cos(x.Psi-x.Psi0)*math.Sin(x.Phi-x.Phi0))*x.Uz,
		Wy: x.Vy + math.Cos(x.Psi-x.Psi0)*math.Cos(x.Theta-x.Theta0)*x.Ux +
			-(math.Cos(x.Psi-x.Psi0)*math.Sin(x.Theta-x.Theta0)*math.Sin(x.Phi-x.Phi0)-
				math.Sin(x.Psi-x.Psi0)*math.Cos(x.Phi-x.Phi0))*x.Uy +
			-(math.Cos(x.Psi-x.Psi0)*math.Sin(x.Theta-x.Theta0)*math.Cos(x.Phi-x.Phi0)+
				math.Sin(x.Psi-x.Psi0)*math.Sin(x.Phi-x.Phi0))*x.Uz,
		Wz: x.Vz + math.Sin(x.Theta-x.Theta0)*x.Ux +
			math.Cos(x.Theta-x.Theta0)*math.Sin(x.Phi-x.Phi0)*x.Uy +
			math.Cos(x.Theta-x.Theta0)*math.Cos(x.Phi-x.Phi0)*x.Uz,
		T: uint32(t * 1000 + 0.5),
	}
	return m, nil
}

// Data to define a piecewise-linear turn, with entry and exit
var airspeed = 120.0                                            // Nice airspeed for maneuvers, kts
var bank = math.Atan((2 * math.Pi * airspeed) / (ahrs.G * 120)) // Bank angle for std rate turn at given airspeed
var sit_turn_def Situation = Situation{                         // start, initiate roll-in, end roll-in, initiate roll-out, end roll-out, end
	t:      []float64{0, 10, 15, 135, 140, 150},
	ux:     []float64{airspeed, airspeed, airspeed, airspeed, airspeed, airspeed},
	uy:     []float64{0, 0, 0, 0, 0, 0},
	uz:     []float64{0, 0, 0, 0, 0, 0},
	phi:    []float64{0, 0, bank, bank, 0, 0},
	theta:  []float64{0, 0, math.Pi / 90, math.Pi / 90, 0, 0},
	psi:    []float64{0, 0, 0, 2 * math.Pi, 2 * math.Pi, 2 * math.Pi},
	phi0:   []float64{0, 0, 0, 0, 0, 0},
	theta0: []float64{0, 0, 0, 0, 0, 0},
	psi0:   []float64{0, 0, 0, 0, 0, 0},
	vx:     []float64{3, 3, 3, 3, 3, 3},
	vy:     []float64{4, 4, 4, 4, 4, 4},
	vz:     []float64{0, 0, 0, 0, 0, 0},
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

	s := ahrs.X0	// Initialize Kalman with a sensible starting state
	fmt.Println("Running Simulation")
	for t := sit_turn_def.t[0]; t < sit_turn_def.t[len(sit_turn_def.t)-1]; t += DT {
		s0, err := sit_turn_def.interpolate(t)
		if err != nil {
			fmt.Printf("Error interpolating at time %f: %s", t, err.Error())
			panic(err)
		}
		fmt.Fprintf(fActual, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
			float64(s0.T)/1000, s0.Ux, s0.Uy, s0.Uz, s0.Phi, s0.Theta, s0.Psi,
			s0.Phi0, s0.Theta0, s0.Psi0, s0.Vx, s0.Vy, s0.Vz)

		c, err := sit_turn_def.control(t)
		if err != nil {
			fmt.Printf("Error calculating control value at time %f: %s", t, err.Error())
			panic(err)
		}
		fmt.Fprintf(fControl, "%f,%f,%f,%f,%f,%f,%f\n",
			float64(c.T)/1000, c.P, c.Q, c.R, c.Ax, c.Ay, c.Az)
		s.Predict(c, ahrs.VX)
		fmt.Fprintf(fPredict, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
			float64(s.T)/1000, s.Ux, s.Uy, s.Uz, s.Phi, s.Theta, s.Psi,
			s.Phi0, s.Theta0, s.Psi0, s.Vx, s.Vy, s.Vz)

		m, err := sit_turn_def.measurement(t)
		if err != nil {
			fmt.Printf("Error calculating measurement value at time %f: %s", t, err.Error())
			panic(err)
		}
		fmt.Fprintf(fMeas, "%f,%f,%f,%f\n",
			float64(m.T)/1000, m.Wx, m.Wy, m.Wz)
		s.Update(m, ahrs.VM)
		fmt.Fprintf(fKalman, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
			float64(s.T)/1000, s.Ux, s.Uy, s.Uz, s.Phi, s.Theta, s.Psi,
			s.Phi0, s.Theta0, s.Psi0, s.Vx, s.Vy, s.Vz)
		fmt.Fprintf(fVar, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
			float64(s.T)/1000, math.Sqrt(s.M.Get(0,0)), math.Sqrt(s.M.Get(1,1)), math.Sqrt(s.M.Get(2,2)),
			math.Sqrt(s.M.Get(3,3)), math.Sqrt(s.M.Get(4,4)), math.Sqrt(s.M.Get(5,5)),
			math.Sqrt(s.M.Get(6,6)), math.Sqrt(s.M.Get(7,7)), math.Sqrt(s.M.Get(8,8)),
			math.Sqrt(s.M.Get(9,9)), math.Sqrt(s.M.Get(10,10)), math.Sqrt(s.M.Get(11,11)),
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
