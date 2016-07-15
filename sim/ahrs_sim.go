// Test out the AHRS code in ahrs/ahrs.go.
// Define a flight path/attitude in code, and then synthesize the matching GPS, gyro, accel (and other) data
// Add some noise if desired.
// Then see if the AHRS code can replicate the "true" attitude given the noisy and limited input data
package main

import (
	"errors"
	"flag"
	"fmt"
	"math"
	"math/rand"
	"net/http"
	"os"
	"sort"
	"strings"

	"github.com/skelterjohn/go.matrix"
	"github.com/westphae/goflying/ahrs"
	"strconv"
)

const pi = math.Pi

// Situation defines a scenario by piecewise-linear interpolation
type Situation struct {
	t                  []float64 // times for situation, s
	u1, u2, u3         []float64 // airspeed, kts, aircraft frame [F/B, R/L, and U/D]
	phi, theta, psi    []float64 // attitude, rad [roll R/L, pitch U/D, heading N->E->S->W]
	phi0, theta0, psi0 []float64 // base attitude, rad [adjust for position of stratux on glareshield]
	v1, v2, v3         []float64 // windspeed, kts, earth frame [N/S, E/W, and U/D]
	m1, m2, m3         []float64 // magnetometer reading
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

	return ahrs.State{
		U1: f*s.u1[ix] + (1-f)*s.u1[ix+1],
		U2: f*s.u2[ix] + (1-f)*s.u2[ix+1],
		U3: f*s.u3[ix] + (1-f)*s.u3[ix+1],
		E0: e0 / ee,
		E1: e1 / ee,
		E2: e2 / ee,
		E3: e3 / ee,
		F0: f0 / ff,
		F1: f1 / ff,
		F2: f2 / ff,
		F3: f3 / ff,
		V1: f*s.v1[ix] + (1-f)*s.v1[ix+1],
		V2: f*s.v2[ix] + (1-f)*s.v2[ix+1],
		V3: f*s.v3[ix] + (1-f)*s.v3[ix+1],
		M1: f*s.m1[ix] + (1-f)*s.m1[ix+1],
		M2: f*s.m2[ix] + (1-f)*s.m2[ix+1],
		M3: f*s.m3[ix] + (1-f)*s.m3[ix+1],
		T:  int64(t*1000000000 + 0.5), // easy rounding for uint
		M:  matrix.DenseMatrix{},
	}, nil
}

// Determine time derivative of an ahrs.State from a Situation definition at a given time
func (s *Situation) derivative(t float64) (ahrs.State, error) {
	if t < s.t[0] || t > s.t[len(s.t)-1] {
		return ahrs.State{}, errors.New("requested time is outside of scenario")
	}

	var t0, t1, ddt float64
	ddt = 0.001
	t0, t1 = t, t+ddt
	if t1 > s.t[len(s.t)-1] {
		t1 = s.t[len(s.t)-1]
		t0 = t1 - ddt
	}

	s0, _ := s.interpolate(t0)
	s1, _ := s.interpolate(t1)

	return ahrs.State{
		U1: (s1.U1 - s0.U1) / ddt,
		U2: (s1.U2 - s0.U2) / ddt,
		U3: (s1.U3 - s0.U3) / ddt,
		E0: (s1.E0 - s0.E0) / ddt,
		E1: (s1.E1 - s0.E1) / ddt,
		E2: (s1.E2 - s0.E2) / ddt,
		E3: (s1.E3 - s0.E3) / ddt,
		F0: (s1.F0 - s0.F0) / ddt,
		F1: (s1.F1 - s0.F1) / ddt,
		F2: (s1.F2 - s0.F2) / ddt,
		F3: (s1.F3 - s0.F3) / ddt,
		V1: (s1.V1 - s0.V1) / ddt,
		V2: (s1.V2 - s0.V2) / ddt,
		V3: (s1.V3 - s0.V3) / ddt,
		M1: (s1.M1 - s0.M1) / ddt,
		M2: (s1.M2 - s0.M2) / ddt,
		M3: (s1.M3 - s0.M3) / ddt,
		T:  int64(t*1000000000 + 0.5), // easy rounding for uint
		M:  matrix.DenseMatrix{},
	}, nil
}

// Determine ahrs.Control variables from a Situation definition at a given time
// gyro noise (Gaussian stdev) and bias are in deg/s
// accel noise and bias are in G
func (s *Situation) control(
		t float64,
		gn, an float64,
		gb, ab []float64,
	) (ahrs.Control, error) {
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

	h1 := 2 * (dx.E1*x.E0 - dx.E0*x.E1 + dx.E3*x.E2 - dx.E2*x.E3)
	h2 := 2 * (dx.E2*x.E0 - dx.E3*x.E1 - dx.E0*x.E2 + dx.E1*x.E3)
	h3 := 2 * (dx.E3*x.E0 + dx.E2*x.E1 - dx.E1*x.E2 - dx.E0*x.E3)

	y1 := -2*(+x.E0*x.E2 + x.E3*x.E1)       + (-dx.U1 + h2*x.U3 - h3*x.U2)/ahrs.G
	y2 := -2*(-x.E0*x.E1 + x.E3*x.E2)       + (-dx.U2 + h3*x.U1 - h1*x.U3)/ahrs.G
	y3 := -2*(+x.E0*x.E0 + x.E3*x.E3 - 0.5) + (-dx.U3 + h1*x.U2 - h2*x.U1)/ahrs.G

	c := ahrs.Control{
		H1: h1*f11 + h2*f12 + h3*f13 + gb[0] + gn * rand.NormFloat64(),
		H2: h1*f21 + h2*f22 + h3*f23 + gb[1] + gn * rand.NormFloat64(),
		H3: h1*f31 + h2*f32 + h3*f33 + gb[2] + gn * rand.NormFloat64(),
		A1: y1*f11 + y2*f12 + y3*f13 + ab[0] + an * rand.NormFloat64(),
		A2: y1*f21 + y2*f22 + y3*f23 + ab[1] + an * rand.NormFloat64(),
		A3: y1*f31 + y2*f32 + y3*f33 + ab[2] + an * rand.NormFloat64(),
		T:  int64(t*1000000000 + 0.5),
	}
	return c, nil
}

// Determine ahrs.Measurement variables from a Situation definition at a given time
// gps noise (gaussian stdev) and bias are in kt
// airspeed noise and bias are in kt
// magnetometer noise and bias are in uT
func (s *Situation) measurement(
		t float64,
		wValid, uValid, mValid bool,
		wn, un, mn, ab float64, mb []float64,
	) (ahrs.Measurement, error) {
	if t < s.t[0] || t > s.t[len(s.t)-1] {
		return ahrs.Measurement{}, errors.New("requested time is outside of scenario")
	}
	x, _ := s.interpolate(t)

	var m = new(ahrs.Measurement)
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
	return *m, nil
}

// Data to define a piecewise-linear turn, with entry and exit
var airspeed = 120.0                                            // Nice airspeed for maneuvers, kts
var bank = math.Atan((2 * pi * airspeed) / (ahrs.G * 120))	// Bank angle for std rate turn at given airspeed
var mush = -airspeed*math.Sin(pi/90)/math.Cos(bank)		// Mush in a turn to maintain altitude
// start, initiate roll-in, end roll-in, initiate roll-out, end roll-out, end
var sitTurnDef = Situation{
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
var sitTakeoffDef = Situation{
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

type AHRSLogger struct {
	f	*os.File
	h	[]string
	fmt 	string
}

func NewAHRSLogger(fn string, h ...string) (AHRSLogger) {
	var l = new(AHRSLogger)

	l.h = h
	f, err := os.Create(fn)
	l.f = f
	if err != nil {
		panic(err)
	}

	fmt.Fprint(l.f, strings.Join(l.h, ","), "\n")
	s := strings.Repeat("%f,", len(l.h))
	l.fmt = strings.Join([]string{s[:len(s)-1], "\n"}, "")
	return *l
}

func (l *AHRSLogger) Log(v ...interface{}) {
	fmt.Fprintf(l.f, l.fmt, v...)
}

func (l *AHRSLogger) Close() {
	l.f.Close()
}

func parseFloatArrayString(str string, a *[]float64) error {
	var err error
	for i, s := range strings.Split(str, ",") {
		(*a)[i], err = strconv.ParseFloat(s, 64)
		if err != nil {
			break
		}
	}
	return err
}

func main() {
	// Handle some shell arguments
	var (
		pdt, udt 						float64
		gyroBiasStr, accelBiasStr, magBiasStr			string
		gyroNoise, accelNoise, gpsNoise, asiNoise, magNoise	float64
		asiBias							float64
		gyroBias, accelBias, magBias				[]float64
		gpsInop, magInop, asiInop				bool
		scenario						string
		sit							Situation
	)
	gyroBias = make([]float64, 3)
	accelBias = make([]float64, 3)
	magBias = make([]float64, 3)

	const (
		defaultPdt = 0.1
		pdtUsage = "Kalman filter predict period, seconds"
		defaultUdt = 0.1
		udtUsage = "Kalman filter update period, seconds"
		defaultGyroNoise = 0.0
		gyroNoiseUsage = "Amount of noise to add to gyro measurements, deg/s"
		defaultGyroBias = "0,0,0"
		gyroBiasUsage = "Amount of bias to add to gyro measurements, \"x,y,z\" deg/s"
		defaultAccelNoise = 0.0
		accelNoiseUsage = "Amount of noise to add to accel measurements, G"
		defaultAccelBias = "0,0,0"
		accelBiasUsage = "Amount of bias to add to accel measurements, \"x,y,z\" G"
		defaultGPSNoise = 0.0
		gpsNoiseUsage = "Amount of noise to add to GPS speed measurements, kt"
		defaultASINoise = 0.0
		asiNoiseUsage = "Amount of noise to add to airspeed measurements, kt"
		defaultASIBias = 0.0
		asiBiasUsage = "Amount of bias to add to airspeed measurements, kt"
		defaultMagNoise = 0.0
		magNoiseUsage = "Amount of noise to add to magnetometer measurements, uT"
		defaultMagBias = "0,0,0"
		magBiasUsage = "Amount of bias to add to magnetometer measurements, \"x,y,z\" uT"
		defaultGPSInop = false
		gpsInopUsage = "Make the GPS inoperative"
		defaultASIInop = true
		asiInopUsage = "Make the Airspeed sensor inoperative"
		defaultMagInop = false
		magInopUsage = "Make the Magnetometer inoperative"
		defaultScenario = "takeoff"
		scenarioUsage = "Scenario to use: \"takeoff\" or \"turn\""
	)

	flag.Float64Var(&pdt, "pdt", defaultPdt, pdtUsage)
	flag.Float64Var(&udt, "udt", defaultUdt, udtUsage)
	flag.Float64Var(&gyroNoise, "gyro-noise", defaultGyroNoise, gyroNoiseUsage)
	flag.Float64Var(&gyroNoise, "g", defaultGyroNoise, gyroNoiseUsage)
	flag.StringVar(&gyroBiasStr, "gyro-bias", defaultGyroBias, gyroBiasUsage)
	flag.StringVar(&gyroBiasStr, "h", defaultGyroBias, gyroBiasUsage)
	flag.Float64Var(&accelNoise, "accel-noise", defaultAccelNoise, accelNoiseUsage)
	flag.Float64Var(&accelNoise, "a", defaultAccelNoise, accelNoiseUsage)
	flag.StringVar(&accelBiasStr, "accel-bias", defaultAccelBias, accelBiasUsage)
	flag.StringVar(&accelBiasStr, "i", defaultAccelBias, accelBiasUsage)
	flag.Float64Var(&gpsNoise, "gps-noise", defaultGPSNoise, gpsNoiseUsage)
	flag.Float64Var(&gpsNoise, "n", defaultGPSNoise, gpsNoiseUsage)
	flag.Float64Var(&asiNoise, "asi-noise", defaultASINoise, asiNoiseUsage)
	flag.Float64Var(&asiNoise, "v", defaultASINoise, asiNoiseUsage)
	flag.Float64Var(&asiBias, "asi-bias", defaultASIBias, asiBiasUsage)
	flag.Float64Var(&asiBias, "j", defaultASIBias, asiBiasUsage)
	flag.Float64Var(&magNoise, "mag-noise", defaultMagNoise, magNoiseUsage)
	flag.Float64Var(&magNoise, "b", defaultMagNoise, magNoiseUsage)
	flag.StringVar(&magBiasStr, "mag-bias", defaultMagBias, magBiasUsage)
	flag.StringVar(&magBiasStr, "k", defaultMagBias, magBiasUsage)
	flag.BoolVar(&gpsInop, "w", defaultGPSInop, gpsInopUsage)
	flag.BoolVar(&asiInop, "u", defaultASIInop, asiInopUsage)
	flag.BoolVar(&magInop, "m", defaultMagInop, magInopUsage)
	flag.StringVar(&scenario, "scenario", defaultScenario, scenarioUsage)
	flag.StringVar(&scenario, "s", defaultScenario, scenarioUsage)
	flag.Parse()

	if err := parseFloatArrayString(gyroBiasStr, &gyroBias); err != nil {
		fmt.Printf("Error %v parsing %s\n", err, gyroBiasStr)
	}
	if err := parseFloatArrayString(accelBiasStr, &accelBias); err != nil {
		fmt.Printf("Error %v parsing %s\n", err, accelBiasStr)
	}
	if err := parseFloatArrayString(magBiasStr, &magBias); err != nil {
		fmt.Printf("Error %v parsing %s\n", err, magBiasStr)
	}

	fmt.Println("Simulation parameters:")
	fmt.Println("Timing:")
	fmt.Printf("\tPredict Freqency: %d Hz\n", int(1/pdt))
	fmt.Printf("\tUpdate  Freqency: %d Hz\n", int(1/udt))
	fmt.Println("Accelerometer:")
	fmt.Printf("\tNoise: %f G\n", accelNoise)
	fmt.Printf("\tBias: %f,%f,%f\n", accelBias[0], accelBias[1], accelBias[2])
	fmt.Println("Gyro:")
	fmt.Printf("\tNoise: %f G\n", gyroNoise)
	fmt.Printf("\tBias: %f,%f,%f\n", gyroBias[0], gyroBias[1], gyroBias[2])
	fmt.Println("GPS:")
	fmt.Printf("\tInop: %t\n", gpsInop)
	fmt.Printf("\tNoise: %f kt\n", gpsNoise)
	fmt.Println("ASI:")
	fmt.Printf("\tInop: %t\n", asiInop)
	fmt.Printf("\tNoise: %f kt\n", asiNoise)
	fmt.Println("Magnetometer:")
	fmt.Printf("\tInop: %t\n", magInop)
	fmt.Printf("\tNoise: %f G\n", magNoise)
	fmt.Printf("\tBias: %f,%f,%f\n", magBias[0], magBias[1], magBias[2])

	gyroNoise *= pi/180
	gyroBias[0] *= pi/180
	gyroBias[1] *= pi/180
	gyroBias[2] *= pi/180

	// Files to save data to for analysis
	lActual := NewAHRSLogger("k_state.csv",
		"T", "Ux", "Uy", "Uz", "Phi", "Theta", "Psi", "Vx", "Vy", "Vz", "Mx", "My", "Mz")
	lKalman := NewAHRSLogger("k_kalman.csv",
		"T", "Ux", "Uy", "Uz", "Phi", "Theta", "Psi", "Vx", "Vy", "Vz", "Mx", "My", "Mz")
	lPredict := NewAHRSLogger("k_predict.csv",
		"T", "Ux", "Uy", "Uz", "Phi", "Theta", "Psi", "Vx", "Vy", "Vz", "Mx", "My", "Mz")
	lVar := NewAHRSLogger("k_var.csv",
		"T", "Ux", "Uy", "Uz", "Phi", "Theta", "Psi", "Vx", "Vy", "Vz", "Mx", "My", "Mz")
	lControl := NewAHRSLogger("k_control.csv",
		"T", "P", "Q", "R", "Ax", "Ay", "Az")
	lMeas := NewAHRSLogger("k_meas.csv",
		"T", "Wx", "Wy", "Wz", "Mx", "My", "Mz", "Ux", "Uy", "Uz")
	lPMeas := NewAHRSLogger("k_predmeas.csv",
		"T", "Wx", "Wy", "Wz", "Mx", "My", "Mz", "Ux", "Uy", "Uz")

	switch scenario {
	case "takeoff":
		sit = sitTakeoffDef
	case "turn":
		sit = sitTurnDef
	default:
		sit = sitTurnDef
	}

	// This is where it all happens
	fmt.Println("Running Simulation")
	var s = ahrs.State{}
	var t, tNextUpdate float64
	t = sit.t[0];
	tNextUpdate = t + udt
	for t < sit.t[len(sit.t)-1] {
		if t>tNextUpdate-1e-9 {
			t = tNextUpdate
		}

		// Peek behind the curtain: the "actual" state, which the algorithm doesn't know
		s0, err := sit.interpolate(t)
		if err != nil {
			fmt.Printf("Error interpolating at time %f: %s", t, err.Error())
			panic(err)
		}
		phi, theta, psi := ahrs.FromQuaternion(s0.E0, s0.E1, s0.E2, s0.E3)
		lActual.Log(float64(s0.T)/1000000000, s0.U1, s0.U2, s0.U3, phi, theta, psi,
			s0.V1, s0.V2, s0.V3, s0.M1, s0.M2, s0.M3)

		// Take control "measurements"
		c, err := sit.control(t, gyroNoise, accelNoise, gyroBias, accelBias)
		if err != nil {
			fmt.Printf("Error calculating control value at time %f: %s", t, err.Error())
			panic(err)
		}
		lControl.Log(float64(c.T)/1000000000, -c.H1, c.H2, c.H3, c.A1, c.A2, c.A3)

		// Take sensor measurements
		m, err := sit.measurement(t, !gpsInop, !asiInop, !magInop, gpsNoise, asiNoise, magNoise, asiBias, magBias)
		if err != nil {
			fmt.Printf("Error calculating measurement value at time %f: %s", t, err.Error())
			panic(err)
		}
		lMeas.Log(float64(m.T)/1000000000, m.W1, m.W2, m.W3, m.M1, m.M2, m.M3, m.U1, m.U2, m.U3)

		// Try to initialize
		if !s.Initialized {
			s.Initialize(m, c)
		}
		// If aircraft frame is inertial, then check calibration
		if s.IsInertial(c, m) && s.Initialized {
			s.Calibrate(c, m)
		}

		// If we have calibration and the Kalman filter is initialized, then run the filter
		if s.Initialized && s.Calibrated {
			// Predict stage of Kalman filter
			s.Predict(c)
			phi, theta, psi = ahrs.FromQuaternion(s.E0, s.E1, s.E2, s.E3)
			lPredict.Log(float64(s.T) / 1000000000, s.U1, s.U2, s.U3, phi, theta, psi,
				s.V1, s.V2, s.V3, s.M1, s.M2, s.M3)

			pm := s.PredictMeasurement()
			lPMeas.Log(float64(m.T) / 1000000000, pm.W1, pm.W2, pm.W3, pm.M1, pm.M2, pm.M3, pm.U1, pm.U2, pm.U3)

			// Update stage of Kalman filter
			if t > tNextUpdate - 1e-9 {
				tNextUpdate += udt
				s.Update(m)
			}
			phi, theta, psi = ahrs.FromQuaternion(s.E0, s.E1, s.E2, s.E3)
			dphi, dtheta, dpsi := ahrs.VarFromQuaternion(s.E0, s.E1, s.E2, s.E3,
				math.Sqrt(s.M.Get(3, 3)), math.Sqrt(s.M.Get(4, 4)),
				math.Sqrt(s.M.Get(5, 5)), math.Sqrt(s.M.Get(6, 6)))
			lKalman.Log(float64(s.T) / 1000000000, s.U1, s.U2, s.U3, phi, theta, psi,
				s.V1, s.V2, s.V3, s.M1, s.M2, s.M3)
			lVar.Log(float64(s.T) / 1000000000,
				math.Sqrt(s.M.Get(0, 0)), math.Sqrt(s.M.Get(1, 1)), math.Sqrt(s.M.Get(2, 2)),
				dphi, dtheta, dpsi,
				math.Sqrt(s.M.Get(7, 7)), math.Sqrt(s.M.Get(8, 8)), math.Sqrt(s.M.Get(9, 9)),
				math.Sqrt(s.M.Get(10, 10)), math.Sqrt(s.M.Get(11, 11)), math.Sqrt(s.M.Get(12, 12)),
			)
		} else {
			lPredict.Log(t, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
			lPMeas.Log(t, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
			lKalman.Log(t, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
			lVar.Log(t, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
			tNextUpdate += pdt
		}
		t += pdt
	}

	// Clean up
	lActual.Close()
	lKalman.Close()
	lPredict.Close()
	lControl.Close()
	lVar.Close()
	lMeas.Close()

	// Run analysis web server
	fmt.Println("Serving charts")
	http.Handle("/", http.FileServer(http.Dir("./")))
	http.ListenAndServe(":8080", nil)
}
