package main

import (
	"github.com/westphae/goflying/ahrs"
	"errors"
	"sort"
	"math"
	"math/rand"
	"github.com/skelterjohn/go.matrix"
)

const pi = math.Pi

// Situation defines a scenario by piecewise-linear interpolation
type SituationSim struct {
	t                  []float64 // times for situation, s
	u1, u2, u3         []float64 // airspeed, kts, aircraft frame [F/B, R/L, and U/D]
	phi, theta, psi    []float64 // attitude, rad [roll R/L, pitch U/D, heading N->E->S->W]
	phi0, theta0, psi0 []float64 // base attitude, rad [adjust for position of stratux on glareshield]
	v1, v2, v3         []float64 // windspeed, kts, earth frame [N/S, E/W, and U/D]
	m1, m2, m3         []float64 // magnetometer reading
}

// BeginTime returns the time stamp when the simulation begins
func (s *SituationSim) BeginTime() (float64) {
	return s.t[0]
}

// Interpolate an ahrs.State from a Situation definition at a given time
func (s *SituationSim) Interpolate(t float64, st *ahrs.State, aBias, bBias, mBias []float64) (error) {
	if t < s.t[0] || t > s.t[len(s.t)-1] {
		st = new(ahrs.State)
		return errors.New("requested time is outside of scenario")
	}
	ix := 0
	if t > s.t[0] {
		ix = sort.SearchFloat64s(s.t, t) - 1
	}

	ddt := (s.t[ix+1] - s.t[ix])
	f := (s.t[ix+1] - t) / ddt
	e0, e1, e2, e3 := ahrs.ToQuaternion(
		f*s.phi[ix]+(1-f)*s.phi[ix+1],
		f*s.theta[ix]+(1-f)*s.theta[ix+1],
		f*s.psi[ix]+(1-f)*s.psi[ix+1])
	ee := math.Sqrt(e0*e0 + e1*e1 + e2*e2 + e3*e3)
	f0, f1, f2, f3 := ahrs.ToQuaternion(
		f*s.phi0[ix]+(1-f)*s.phi0[ix+1],
		f*s.theta0[ix]+(1-f)*s.theta0[ix+1],
		f*s.psi0[ix]+(1-f)*s.psi0[ix+1])
	ff := math.Sqrt(f0*f0 + f1*f1 + f2*f2 + f3*f3)

	// U, Z, E, H, N,
	// V, C, F, D, L
	st.U1 = f*s.u1[ix] + (1-f)*s.u1[ix+1]
	st.U2 = f*s.u2[ix] + (1-f)*s.u2[ix+1]
	st.U3 = f*s.u3[ix] + (1-f)*s.u3[ix+1]
	st.Z1 = (s.u1[ix+1] - s.u1[ix]) / ddt
	st.Z2 = (s.u2[ix+1] - s.u2[ix]) / ddt
	st.Z3 = (s.u3[ix+1] - s.u3[ix]) / ddt
	st.E0 = e0 / ee
	st.E1 = e1 / ee
	st.E2 = e2 / ee
	st.E3 = e3 / ee
	st.H1 = 0 //TODO westphae: calc these
	st.H2 = 0
	st.H3 = 0
	st.N1 = f*s.m1[ix] + (1-f)*s.m1[ix+1]
	st.N2 = f*s.m2[ix] + (1-f)*s.m2[ix+1]
	st.N3 = f*s.m3[ix] + (1-f)*s.m3[ix+1]
	st.V1 = f*s.v1[ix] + (1-f)*s.v1[ix+1]
	st.V2 = f*s.v2[ix] + (1-f)*s.v2[ix+1]
	st.V3 = f*s.v3[ix] + (1-f)*s.v3[ix+1]
	st.C1 = aBias[0]
	st.C2 = aBias[1]
	st.C3 = aBias[2]
	st.F0 = f0 / ff
	st.F1 = f1 / ff
	st.F2 = f2 / ff
	st.F3 = f3 / ff
	st.D1 = bBias[0]
	st.D2 = bBias[1]
	st.D3 = bBias[2]
	st.L1 = mBias[0]
	st.L2 = mBias[1]
	st.L3 = mBias[2]
	st.T =  int64(t*1000000000 + 0.5) // easy rounding for uint
	st.M =  new(matrix.DenseMatrix)
	return nil
}

// Determine time derivative of an ahrs.State from a Situation definition at a given time
func (s *SituationSim) derivative(t float64, st *ahrs.State) (error) {
	if t < s.t[0] || t > s.t[len(s.t)-1] {
		st = new(ahrs.State)
		return errors.New("requested time is outside of scenario")
	}

	var t0, t1, ddt float64
	ddt = 0.001
	t0, t1 = t, t+ddt
	if t1 > s.t[len(s.t)-1] {
		t1 = s.t[len(s.t)-1]
		t0 = t1 - ddt
	}

	var s0, s1 ahrs.State

	s.Interpolate(t0, &s0)
	s.Interpolate(t1, &s1)

	st.U1 = (s1.U1 - s0.U1) / ddt
	st.U2 = (s1.U2 - s0.U2) / ddt
	st.U3 = (s1.U3 - s0.U3) / ddt
	st.E0 = (s1.E0 - s0.E0) / ddt
	st.E1 = (s1.E1 - s0.E1) / ddt
	st.E2 = (s1.E2 - s0.E2) / ddt
	st.E3 = (s1.E3 - s0.E3) / ddt
	st.F0 = (s1.F0 - s0.F0) / ddt
	st.F1 = (s1.F1 - s0.F1) / ddt
	st.F2 = (s1.F2 - s0.F2) / ddt
	st.F3 = (s1.F3 - s0.F3) / ddt
	st.V1 = (s1.V1 - s0.V1) / ddt
	st.V2 = (s1.V2 - s0.V2) / ddt
	st.V3 = (s1.V3 - s0.V3) / ddt
	st.M1 = (s1.M1 - s0.M1) / ddt
	st.M2 = (s1.M2 - s0.M2) / ddt
	st.M3 = (s1.M3 - s0.M3) / ddt
	st.T =  int64(t*1000000000 + 0.5) // easy rounding for uint
	st.M =  new(matrix.DenseMatrix)
	return nil
}

// Determine ahrs.Control variables from a Situation definition at a given time
// gyro noise (Gaussian stdev) and bias are in deg/s
// accel noise and bias are in G
func (s *SituationSim) Control(t float64, c *ahrs.Control, gn, an float64, gb, ab []float64, ) (error) {
	var x, dx ahrs.State
	erri := s.Interpolate(t, &x)
	errd := s.derivative(t, &dx)
	if erri != nil || errd != nil {
		c = new(ahrs.Control)
		return errors.New("requested time is outside of scenario")
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

	h1 := 2 * (dx.E1*x.E0 - dx.E0*x.E1 + dx.E3*x.E2 - dx.E2*x.E3)
	h2 := 2 * (dx.E2*x.E0 - dx.E3*x.E1 - dx.E0*x.E2 + dx.E1*x.E3)
	h3 := 2 * (dx.E3*x.E0 + dx.E2*x.E1 - dx.E1*x.E2 - dx.E0*x.E3)

	y1 := -2*(+x.E0*x.E2 + x.E3*x.E1)       + (-dx.U1 + h2*x.U3 - h3*x.U2)/ahrs.G
	y2 := -2*(-x.E0*x.E1 + x.E3*x.E2)       + (-dx.U2 + h3*x.U1 - h1*x.U3)/ahrs.G
	y3 := -2*(+x.E0*x.E0 + x.E3*x.E3 - 0.5) + (-dx.U3 + h1*x.U2 - h2*x.U1)/ahrs.G

	c.H1 = h1*f11 + h2*f12 + h3*f13 + (gb[0] + gn * rand.NormFloat64())*pi/180
	c.H2 = h1*f21 + h2*f22 + h3*f23 + (gb[1] + gn * rand.NormFloat64())*pi/180
	c.H3 = h1*f31 + h2*f32 + h3*f33 + (gb[2] + gn * rand.NormFloat64())*pi/180
	c.A1 = y1*f11 + y2*f12 + y3*f13 +  ab[0] + an * rand.NormFloat64()
	c.A2 = y1*f21 + y2*f22 + y3*f23 +  ab[1] + an * rand.NormFloat64()
	c.A3 = y1*f31 + y2*f32 + y3*f33 +  ab[2] + an * rand.NormFloat64()
	c.T =  int64(t*1000000000 + 0.5)
	return nil
}

// Determine ahrs.Measurement variables from a Situation definition at a given time
// gps noise (gaussian stdev) and bias are in kt
// airspeed noise and bias are in kt
// magnetometer noise and bias are in uT
func (s *SituationSim) Measurement(t float64, m *ahrs.Measurement, wValid, uValid, mValid bool,
		wn, un, mn, ab float64, mb []float64) (error) {
	if t < s.t[0] || t > s.t[len(s.t)-1] {
		m = new(ahrs.Measurement)
		return errors.New("requested time is outside of scenario")
	}

	var x ahrs.State
	s.Interpolate(t, &x)

	if wValid {
		m.WValid = true
		m.W1 = x.U1 * 2 * (+x.E0 * x.E0 + x.E1 * x.E1 - 0.5) +
			x.U2 * 2 * (+x.E0 * x.E3 + x.E1 * x.E2) +
			x.U3 * 2 * (-x.E0 * x.E2 + x.E1 * x.E3) +
			x.V1 + wn * rand.NormFloat64()
		m.W2 = x.U1 * 2 * (-x.E0 * x.E3 + x.E2 * x.E1) +
			x.U2 * 2 * (+x.E0 * x.E0 + x.E2 * x.E2 - 0.5) +
			x.U3 * 2 * (+x.E0 * x.E1 + x.E2 * x.E3) +
			x.V2 + wn * rand.NormFloat64()
		m.W3 = x.U1 * 2 * (+x.E0 * x.E2 + x.E3 * x.E1) +
			x.U2 * 2 * (-x.E0 * x.E1 + x.E3 * x.E2) +
			x.U3 * 2 * (+x.E0 * x.E0 + x.E3 * x.E3 - 0.5) +
			x.V3 + wn * rand.NormFloat64()
	}
	if uValid {
		m.UValid = true
		m.U1 = x.U1 + ab + un * rand.NormFloat64()
		// ASI doesn't read U2 or U3, they are here only to bias toward coordinated flight
	}
	if mValid {
		m.MValid = true
		m.M1 = x.M1 * 2 * (+x.E0 * x.E0 + x.E1 * x.E1 - 0.5) +
			x.M2 * 2 * (-x.E0 * x.E3 + x.E1 * x.E2) +
			x.M3 * 2 * (+x.E0 * x.E2 + x.E1 * x.E3) +
			mb[0] + mn * rand.NormFloat64()
		m.M2 = x.M1 * 2 * (+x.E0 * x.E3 + x.E2 * x.E1) +
			x.M2 * 2 * (+x.E0 * x.E0 + x.E2 * x.E2 - 0.5) +
			x.M3 * 2 * (-x.E0 * x.E1 + x.E2 * x.E3) +
			mb[1] + mn * rand.NormFloat64()
		m.M3 = x.M1 * 2 * (-x.E0 * x.E2 + x.E3 * x.E1) +
			x.M2 * 2 * (+x.E0 * x.E1 + x.E3 * x.E2) +
			x.M3 * 2 * (+x.E0 * x.E0 + x.E3 * x.E3 - 0.5) +
			mb[2] + mn * rand.NormFloat64()
	}
	m.T = int64(t*1000000000 + 0.5)
	return nil
}

// Data to define a piecewise-linear turn, with entry and exit
var airspeed = 120.0                                            // Nice airspeed for maneuvers, kts
var bank = math.Atan((2 * pi * airspeed) / (ahrs.G * 120))	// Bank angle for std rate turn at given airspeed
var mush = -airspeed*math.Sin(pi/90)/math.Cos(bank)		// Mush in a turn to maintain altitude
// start, initiate roll-in, end roll-in, initiate roll-out, end roll-out, end
var sitTurnDef = &SituationSim{
	t:      []float64{0, 10, 15, 255, 260, 270},
	u1:     []float64{airspeed, airspeed, airspeed, airspeed, airspeed, airspeed},
	u2:     []float64{0, 0, 0, 0, 0, 0},
	u3:     []float64{0, 0, mush, mush, 0, 0},
	phi:    []float64{0, 0, bank, bank, 0, 0},
	theta:  []float64{0, 0, pi/90, pi/90, 0, 0},
	psi:    []float64{0, 0, 0, 4*pi, 4*pi, 4*pi},
	phi0:   []float64{0, 0, 0, 0, 0, 0},
	theta0: []float64{0, 0, 0, 0, 0, 0},
	psi0:   []float64{pi/2, pi/2, pi/2, pi/2, pi/2, pi/2},
	v1:     []float64{3, 3, 3, 3, 3, 3},
	v2:     []float64{4, 4, 4, 4, 4, 4},
	v3:     []float64{0, 0, 0, 0, 0, 0},
	m1:     []float64{0, 0, 0, 0, 0, 0},
	m2:     []float64{1, 1, 1, 1, 1, 1},
	m3:     []float64{-1, -1, -1, -1, -1, -1},
}

var bank1 = math.Atan((2*pi*95) / (ahrs.G * 120))
var bank2 = math.Atan((2*pi*120) / (ahrs.G * 120))
var sitTakeoffDef = &SituationSim{
	t:	[]float64{   0,   10,   30,   35,   55,  115,    120,    150,    155,    175,    180,    210,  215,  230},
	u1:	[]float64{   9,    9,   68,   83,   95,   95,     95,     95,     95,    120,    120,    120,  120,  140},
	u2:	[]float64{   0,    0,    0,    0,    0,    0,      0,      0,      0,      0,      0,      0,    0,    0},
	u3:	[]float64{   0,    0,    0,   -3,   -3,   -3,     -2,     -2,     -2,      0,      0,      0,    0,    0},
	phi:	[]float64{   0,    0,    0,    0,    0,    0, -bank1, -bank1,      0,      0, -bank2, -bank2,    0,    0},
	theta:	[]float64{   0,    0,    0,  0.2,  0.2,  0.2,   0.12,   0.12,   0.12,   0.03,   0.03,   0.03,    0,    0},
	psi:	[]float64{   0,    0,    0,    0,    0,    0,      0,  -pi/2,  -pi/2,  -pi/2,  -pi/2,    -pi,  -pi,  -pi},
	phi0:	[]float64{   0,    0,    0,    0,    0,    0,      0,      0,      0,      0,      0,      0,    0,    0},
	theta0:	[]float64{   0,    0,    0,    0,    0,    0,      0,      0,      0,      0,      0,      0,    0,    0},
	psi0:	[]float64{pi/2, pi/2, pi/2, pi/2, pi/2, pi/2,   pi/2,   pi/2,   pi/2,   pi/2,   pi/2,   pi/2, pi/2, pi/2},
	v1:	[]float64{   0,    0,    0,    0,    0,    0,      0,      0,      0,      0,      0,      0,    0,    0},
	v2:	[]float64{  -8,   -8,   -8,   -8,   -8,   -8,    -10,    -10,    -10,    -12,    -12,    -12,  -12,  -12},
	v3:	[]float64{   0,    0,    0,    0,    0,    0,      0,      0,      0,      0,      0,      0,    0,    0},
	m1:	[]float64{   0,    0,    0,    0,    0,    0,      0,      0,      0,      0,      0,      0,    0,    0},
	m2:	[]float64{   1,    1,    1,    1,    1,    1,      1,      1,      1,      1,      1,      1,    1,    1},
	m3:	[]float64{  -1,   -1,   -1,   -1,   -1,   -1,     -1,     -1,     -1,     -1,     -1,     -1,   -1,   -1},
}


