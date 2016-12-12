/*
Test out the AHRS code in ahrs/ahrs.go.
Define a flight path/attitude in code, and then synthesize the matching GPS, gyro, accel (and other) data
Add some noise if desired.
Then see if the AHRS code can replicate the "true" attitude given the noisy and limited input data
*/

package main

import (
	"flag"
	"fmt"
	"log"
	"net/http"
	"strings"

	"github.com/westphae/goflying/ahrs"
	"strconv"
	"io/ioutil"
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
		gpsInop, magInop, asiInop       			bool
		algo							string
		s                                                       ahrs.AHRSProvider
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
		gyroNoiseUsage = "Amount of noise to add to gyro measurements, °/s"
		defaultGyroBias = "0,0,0"
		gyroBiasUsage = "Amount of bias to add to gyro measurements, \"x,y,z\" °/s"
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
		magNoiseUsage = "Amount of noise to add to magnetometer measurements, μT"
		defaultMagBias = "0,0,0"
		magBiasUsage = "Amount of bias to add to magnetometer measurements, \"x,y,z\" μT"
		defaultGPSInop = false
		gpsInopUsage = "Make the GPS inoperative"
		defaultASIInop = true
		asiInopUsage = "Make the Airspeed sensor inoperative"
		defaultMagInop = false
		magInopUsage = "Make the Magnetometer inoperative"
		defaultScenario = "takeoff"
		scenarioUsage = "Scenario to use: filename or \"takeoff\" or \"turn\""
		defaultAlgo = "simple"
		algoUsage = "Algo to use for AHRS: simple (default), heuristic, kalman, kalman1, kalman2"
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
	flag.StringVar(&algo, "algo", defaultAlgo, algoUsage)
	flag.Parse()

	switch scenario {
	case "takeoff":
		sit = sitTakeoffDef
	case "turn":
		sit = sitTurnDef
	default:
		log.Printf("Loading data from %s\n", scenario)
		sit, err = NewSituationFromFile(scenario)
		if err != nil {
			log.Fatalln(err)
		}
	}

	s0 := new(ahrs.State)       // Actual state from simulation, for comparison
	m  := ahrs.NewMeasurement() // Measurement from IMU

	fmt.Println("Simulation parameters:")
	switch strings.ToLower(algo) {
	case "simple":
		fmt.Println("Running simple AHRS")
		ioutil.WriteFile("config.json", []byte(ahrs.SimpleJSONConfig), 0644)
		s = ahrs.InitializeSimple(m, "ahrs.csv")
	case "heuristic":
		fmt.Println("Running heuristic AHRS")
		ioutil.WriteFile("config.json", []byte(ahrs.HeuristicJSONConfig), 0644)
		s = ahrs.InitializeHeuristic(m)
	case "kalman":
		fmt.Println("Running Kalman AHRS")
		ioutil.WriteFile("config.json", []byte(ahrs.KalmanJSONConfig), 0644)
		s = ahrs.InitializeKalman(m)
	case "kalman1":
		fmt.Println("Running Kalman1 AHRS")
		ioutil.WriteFile("config.json", []byte(ahrs.Kalman1JSONConfig), 0644)
		s = ahrs.InitializeKalman1(m)
	case "kalman2":
		fmt.Println("Running Kalman2 AHRS")
		ioutil.WriteFile("config.json", []byte(ahrs.Kalman2JSONConfig), 0644)
		s = ahrs.InitializeKalman2(m)
	default:
		fmt.Printf("No such AHRS implementation: %s\n", algo)
		return
	}

	if err := parseFloatArrayString(gyroBiasStr, &gyroBias); err != nil {
		fmt.Printf("Error %v parsing %s\n", err, gyroBiasStr)
		return
	}
	if err := parseFloatArrayString(accelBiasStr, &accelBias); err != nil {
		fmt.Printf("Error %v parsing %s\n", err, accelBiasStr)
		return
	}
	if err := parseFloatArrayString(magBiasStr, &magBias); err != nil {
		fmt.Printf("Error %v parsing %s\n", err, magBiasStr)
		return
	}

	fmt.Println("Timing:")
	fmt.Printf("\tPredict Freqency: %d Hz\n", int(1/pdt))
	fmt.Printf("\tUpdate  Freqency: %d Hz\n", int(1/udt))
	fmt.Println("Accelerometer:")
	fmt.Printf("\tNoise: %f G\n", accelNoise)
	fmt.Printf("\tBias: %f,%f,%f\n", accelBias[0], accelBias[1], accelBias[2])
	fmt.Println("Gyro:")
	fmt.Printf("\tNoise: %f °/s\n", gyroNoise)
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

	uBias := []float64{asiBias, 0, 0}

	// This is where it all happens
	fmt.Println("Running Simulation")
	t := sit.BeginTime()
	tNextUpdate := t + udt
	sit.Measurement(t, m, !asiInop, !gpsInop, true, !magInop,
		asiNoise, gpsNoise, accelNoise, gyroNoise, magNoise,
		uBias, accelBias, gyroBias, magBias)

	for {
		if t>tNextUpdate-1e-9 {
			t = tNextUpdate
		}

		// Peek behind the curtain: the "actual" state, which the algorithm doesn't know
		if err := sit.Interpolate(t, s0, accelBias, gyroBias, magBias); err != nil {
			log.Printf("Interpolation error at time %f: %s\n", t, err)
			break
		}
		//TODO westphae: log actual state

		// Take sensor measurements
		if err := sit.Measurement(t, m, !asiInop, !gpsInop, true, !magInop,
			asiNoise, gpsNoise, accelNoise, gyroNoise, magNoise,
			uBias, accelBias, gyroBias, magBias); err != nil {
			log.Printf("Measurement error at time %f: %s\n", t, err)
			break
		}

		// Predict stage of Kalman filter
		s.Predict(t)

		// Update stage of Kalman filter
		if t > tNextUpdate - 1e-9 {
			tNextUpdate += udt
			s.Update(m)
			log.Printf("Time: %.2f\n", t)
		}

		t += pdt
	}

	// Run analysis web server
	fmt.Println("Serving charts")
	http.Handle("/", http.FileServer(http.Dir("./")))
	http.ListenAndServe(":8080", nil)
}
