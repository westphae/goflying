package main

import (
	"github.com/westphae/goflying/ahrs"
	"errors"
	"sort"
	"math"
	"math/rand"
	"github.com/skelterjohn/go.matrix"
)

const (
	pi = math.Pi
	DEG = 180/pi
)

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
	tz := 1e-6
	fz := (s.t[ix+1] - t - tz) / ddt
	e0, e1, e2, e3 := ahrs.ToQuaternion(
		(f*s.phi[ix]+(1-f)*s.phi[ix+1])/DEG,
		(f*s.theta[ix]+(1-f)*s.theta[ix+1])/DEG,
		(f*s.psi[ix]+(1-f)*s.psi[ix+1])/DEG)
	ee := math.Sqrt(e0*e0 + e1*e1 + e2*e2 + e3*e3)
	ez0, ez1, ez2, ez3 := ahrs.ToQuaternion(
		(fz*s.phi[ix]+(1-fz)*s.phi[ix+1])/DEG,
		(fz*s.theta[ix]+(1-fz)*s.theta[ix+1])/DEG,
		(fz*s.psi[ix]+(1-fz)*s.psi[ix+1])/DEG)
	eez := math.Sqrt(e0*e0 + e1*e1 + e2*e2 + e3*e3)
	f0, f1, f2, f3 := ahrs.ToQuaternion(
		(f*s.phi0[ix]+(1-f)*s.phi0[ix+1])/DEG,
		(f*s.theta0[ix]+(1-f)*s.theta0[ix+1])/DEG,
		(f*s.psi0[ix]+(1-f)*s.psi0[ix+1])/DEG)
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
	dE0 := +(ez0 / eez - st.E0) / tz
	dE1 := -(ez1 / eez - st.E1) / tz
	dE2 := -(ez2 / eez - st.E2) / tz
	dE3 := -(ez3 / eez - st.E3) / tz
	st.H1 = -2*(st.E0*dE1 + st.E1*dE0 + st.E2*dE3 - st.E3*dE2)*DEG
	st.H2 = -2*(st.E0*dE2 - st.E1*dE3 + st.E2*dE0 + st.E3*dE1)*DEG
	st.H3 = -2*(st.E0*dE3 + st.E1*dE2 - st.E2*dE1 + st.E3*dE0)*DEG
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
	st.T =  t
	st.M =  new(matrix.DenseMatrix)
	st.N =  new(matrix.DenseMatrix)
	return nil
}

// Determine ahrs.Measurement variables from a Situation definition at a given time
// gps noise (gaussian stdev) and bias are in kt
// airspeed noise and bias are in kt
// accelerometer noise and bias are in G
// gyro noise and bias are in deg/s
// magnetometer noise and bias are in uT
func (s *SituationSim) Measurement(t float64, m *ahrs.Measurement,
		uValid, wValid, sValid, mValid bool,
		uNoise, wNoise, aNoise, bNoise, mNoise float64,
		uBias, aBias, bBias, mBias []float64,
	) (err error) {
	if t < s.t[0] || t > s.t[len(s.t)-1] {
		m = new(ahrs.Measurement)
		return errors.New("requested time is outside of scenario")
	}

	var x, z ahrs.State
	tz := 1e-6
	s.Interpolate(t, &x, aBias, bBias, mBias)
	s.Interpolate(t+tz, &z, aBias, bBias, mBias)

	dU1 := (z.U1-x.U1)/tz
	dU2 := (z.U2-x.U2)/tz
	dU3 := (z.U3-x.U3)/tz
	dE0 := (z.E0-x.E0)/tz
	dE1 := (z.E1-x.E1)/tz
	dE2 := (z.E2-x.E2)/tz
	dE3 := (z.E3-x.E3)/tz

	// eij rotates earth frame i component into aircraft frame j component
	e11 := 2 * (+x.E0 * x.E0 + x.E1 * x.E1 - 0.5)
	e12 := 2 * (+x.E0 * x.E3 + x.E1 * x.E2)
	e13 := 2 * (-x.E0 * x.E2 + x.E1 * x.E3)
	e21 := 2 * (-x.E0 * x.E3 + x.E2 * x.E1)
	e22 := 2 * (+x.E0 * x.E0 + x.E2 * x.E2 - 0.5)
	e23 := 2 * (+x.E0 * x.E1 + x.E2 * x.E3)
	e31 := 2 * (+x.E0 * x.E2 + x.E3 * x.E1)
	e32 := 2 * (-x.E0 * x.E1 + x.E3 * x.E2)
	e33 := 2 * (+x.E0 * x.E0 + x.E3 * x.E3 - 0.5)

	// fij rotates sensor frame i component into aircraft frame j component
	f11 := 2 * (+x.F0 * x.F0 + x.F1 * x.F1 - 0.5)
	f12 := 2 * (+x.F0 * x.F3 + x.F1 * x.F2)
	f13 := 2 * (-x.F0 * x.F2 + x.F1 * x.F3)
	f21 := 2 * (-x.F0 * x.F3 + x.F2 * x.F1)
	f22 := 2 * (+x.F0 * x.F0 + x.F2 * x.F2 - 0.5)
	f23 := 2 * (+x.F0 * x.F1 + x.F2 * x.F3)
	f31 := 2 * (+x.F0 * x.F2 + x.F3 * x.F1)
	f32 := 2 * (-x.F0 * x.F1 + x.F3 * x.F2)
	f33 := 2 * (+x.F0 * x.F0 + x.F3 * x.F3 - 0.5)

	if uValid { // ASI doesn't read U2 or U3
		m.UValid = true
		m.U1 = x.U1 + uBias[0] + uNoise * rand.NormFloat64()
	}

	if wValid {
		m.WValid = true
		m.W1 = e11*x.U1 + e12*x.U2 + e13*x.U3 + x.V1 + wNoise*rand.NormFloat64()
		m.W2 = e21*x.U1 + e22*x.U2 + e23*x.U3 + x.V2 + wNoise*rand.NormFloat64()
		m.W3 = e31*x.U1 + e32*x.U2 + e33*x.U3 + x.V3 + wNoise*rand.NormFloat64()
	}

	if sValid {
		m.SValid = true
		h1 := 2*(dE1*x.E0 - dE0*x.E1 + dE3*x.E2 - dE2*x.E3)
		h2 := 2*(dE2*x.E0 - dE3*x.E1 - dE0*x.E2 + dE1*x.E3)
		h3 := 2*(dE3*x.E0 + dE2*x.E1 - dE1*x.E2 - dE0*x.E3)

		y1 := -e31 + (-dU1 + h2*x.U3 - h3*x.U2)/ahrs.G
		y2 := -e32 + (-dU2 + h3*x.U1 - h1*x.U3)/ahrs.G
		y3 := -e33 + (-dU3 + h1*x.U2 - h2*x.U1)/ahrs.G

		m.A1 = f11*y1 + f12*y2 + f13*y3 +  aBias[0] + aNoise*rand.NormFloat64()
		m.A2 = f21*y1 + f22*y2 + f23*y3 +  aBias[1] + aNoise*rand.NormFloat64()
		m.A3 = f31*y1 + f32*y2 + f33*y3 +  aBias[2] + aNoise*rand.NormFloat64()

		m.B1 = (f11*h1 + f12*h2 + f13*h3)*DEG + (bBias[0] + bNoise*rand.NormFloat64())
		m.B2 = (f21*h1 + f22*h2 + f23*h3)*DEG + (bBias[1] + bNoise*rand.NormFloat64())
		m.B3 = (f31*h1 + f32*h2 + f33*h3)*DEG + (bBias[2] + bNoise*rand.NormFloat64())
	}

	if mValid {
		m.MValid = true
		m1 := x.N1*e11 + x.N2*e21 + x.N3*e31
		m2 := x.N1*e12 + x.N2*e22 + x.N3*e32
		m3 := x.N1*e13 + x.N2*e23 + x.N3*e33
		m.M1 = f11*m1 + f12*m2 + f13*m3 + mBias[0] + mNoise*rand.NormFloat64()
		m.M2 = f21*m1 + f22*m2 + f23*m3 + mBias[1] + mNoise*rand.NormFloat64()
		m.M3 = f31*m1 + f32*m2 + f33*m3 + mBias[2] + mNoise*rand.NormFloat64()
	}

	m.T = t
	return nil
}

// Data to define a piecewise-linear turn, with entry and exit
var airspeed = 120.0                                            // Nice airspeed for maneuvers, kts
var bank = math.Atan((2 * pi * airspeed) / (ahrs.G * 120))*DEG	// Bank angle for std rate turn at given airspeed
var mush = -airspeed*math.Sin(pi/90)/math.Cos(bank/DEG)		// Mush in a turn to maintain altitude
// start, initiate roll-in, end roll-in, initiate roll-out, end roll-out, end
var sitTurnDef = &SituationSim{
	t:      []float64{0, 10, 15, 255, 260, 270},
	u1:     []float64{airspeed, airspeed, airspeed, airspeed, airspeed, airspeed},
	u2:     []float64{0, 0, 0, 0, 0, 0},
	u3:     []float64{0, 0, mush, mush, 0, 0},
	phi:    []float64{0, 0, bank, bank, 0, 0},
	theta:  []float64{0, 0, 2, 2, 0, 0},
	psi:    []float64{0, 0, 0, 720, 720, 720},
	phi0:   []float64{0, 0, 0, 0, 0, 0},
	theta0: []float64{0, 0, 0, 0, 0, 0},
	psi0:   []float64{90, 90, 90, 90, 90, 90},
	v1:     []float64{3, 3, 3, 3, 3, 3},
	v2:     []float64{4, 4, 4, 4, 4, 4},
	v3:     []float64{0, 0, 0, 0, 0, 0},
	m1:     []float64{0, 0, 0, 0, 0, 0},
	m2:     []float64{1, 1, 1, 1, 1, 1},
	m3:     []float64{-1, -1, -1, -1, -1, -1},
}

var bank1 = math.Atan((2*pi*95) / (ahrs.G * 120))*DEG
var bank2 = math.Atan((2*pi*120) / (ahrs.G * 120))*DEG
var sitTakeoffDef = &SituationSim{
	t:	[]float64{ 0, 10, 30, 35, 55, 115,    120,    150, 155, 175,    180,    210,  215,  230},
	u1:	[]float64{ 9,  9, 68, 83, 95,  95,     95,     95,  95, 120,    120,    120,  120,  140},
	u2:	[]float64{ 0,  0,  0,  0,  0,   0,      0,      0,   0,   0,      0,      0,    0,    0},
	u3:	[]float64{ 0,  0,  0, -3, -3,  -3,     -2,     -2,  -2,   0,      0,      0,    0,    0},
	phi:	[]float64{ 0,  0,  0,  0,  0,   0, -bank1, -bank1,   0,   0, -bank2, -bank2,    0,    0},
	theta:	[]float64{ 0,  0,  0, 10, 10,  10,      5,      5,   5,   2,      2,      2,    0,    0},
	psi:	[]float64{ 0,  0,  0,  0,  0,   0,      0,    -90, -90, -90,    -90,   -180, -180, -180},
	phi0:	[]float64{ 0,  0,  0,  0,  0,   0,      0,      0,   0,   0,      0,      0,    0,    0},
	theta0:	[]float64{ 0,  0,  0,  0,  0,   0,      0,      0,   0,   0,      0,      0,    0,    0},
	psi0:	[]float64{90, 90, 90, 90, 90,  90,     90,     90,  90,  90,     90,     90,   90,   90},
	v1:	[]float64{ 0,  0,  0,  0,  0,   0,      0,      0,   0,   0,      0,      0,    0,    0},
	v2:	[]float64{-8, -8, -8, -8, -8,  -8,    -10,    -10, -10, -12,    -12,    -12,  -12,  -12},
	v3:	[]float64{ 0,  0,  0,  0,  0,   0,      0,      0,   0,   0,      0,      0,    0,    0},
	m1:	[]float64{ 0,  0,  0,  0,  0,   0,      0,      0,   0,   0,      0,      0,    0,    0},
	m2:	[]float64{ 1,  1,  1,  1,  1,   1,      1,      1,   1,   1,      1,      1,    1,    1},
	m3:	[]float64{-1, -1, -1, -1, -1,  -1,     -1,     -1,  -1,  -1,     -1,     -1,   -1,   -1},
}


