// Test out the AHRS code in ahrs/ahrs.go.
// Define a flight path/attitude in code, and then synthesize the matching GPS, gyro, accel (and other) data
// Add some noise if desired.
// Then see if the AHRS code can replicate the "true" attitude given the noisy and limited input data
package main

import (
	"flag"
	"fmt"
	"log"
	"math"
	"net/http"
	"strings"

	"github.com/westphae/goflying/ahrs"
	"strconv"
	"os"
)

func parseFloatArrayString(str string, a *[]float64) (err error) {
	for i, s := range strings.Split(str, ",") {
		(*a)[i], err = strconv.ParseFloat(s, 64)
		if err != nil {
			break
		}
	}
	return
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
		err 							error
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

	gyroNoise *= math.Pi/180
	gyroBias[0] *= math.Pi/180
	gyroBias[1] *= math.Pi/180
	gyroBias[2] *= math.Pi/180

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
	s0 := new(ahrs.State)
	s  := new(ahrs.State)
	c  := new(ahrs.Control)
	m  := new(ahrs.Measurement)
	pm := new(ahrs.Measurement)
	t := sit.BeginTime()
	tNextUpdate := t + udt

	// Try to initialize
	s.Initialize(m, c)

	for {
		if t>tNextUpdate-1e-9 {
			t = tNextUpdate
		}

		// Peek behind the curtain: the "actual" state, which the algorithm doesn't know
		if err := sit.Interpolate(t, s0); err != nil {
			break
		}
		phi, theta, psi := ahrs.FromQuaternion(s0.E0, s0.E1, s0.E2, s0.E3)
		lActual.Log(float64(s0.T)/1000000000, s0.U1, s0.U2, s0.U3, phi, theta, psi,
			s0.V1, s0.V2, s0.V3, s0.M1, s0.M2, s0.M3)

		// Take control "measurements"
		if err := sit.Control(t, c, gyroNoise, accelNoise, gyroBias, accelBias); err != nil {
			break
		}
		lControl.Log(float64(c.T)/1000000000, -c.H1, c.H2, c.H3, c.A1, c.A2, c.A3)

		// Take sensor measurements
		if err := sit.Measurement(t, m, !gpsInop, !asiInop, !magInop, gpsNoise, asiNoise, magNoise, asiBias, magBias); err != nil {
			break
		}
		lMeas.Log(float64(m.T)/1000000000, m.W1, m.W2, m.W3, m.M1, m.M2, m.M3, m.U1, m.U2, m.U3)

		// If aircraft frame is inertial, then check calibration
		if s.IsInertial(c, m) {
			s.Calibrate(c, m)
		}

		// If we have calibration and the Kalman filter is initialized, then run the filter
		if s.Calibrated {
			// Predict stage of Kalman filter
			s.Predict(c)
			phi, theta, psi = ahrs.FromQuaternion(s.E0, s.E1, s.E2, s.E3)
			lPredict.Log(float64(s.T) / 1000000000, s.U1, s.U2, s.U3, phi, theta, psi,
				s.V1, s.V2, s.V3, s.M1, s.M2, s.M3)

			s.PredictMeasurement(pm)
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

		// Apply some heuristics
		if s.U1 < 0 {
			s.U1 = -s.U1
			s.V1 = -s.V1
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
