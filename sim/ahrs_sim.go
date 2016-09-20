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

	// Files to save data to for analysis
	// U, Z, E, H, N,
	// V, C, F, D, L
	lActual := ahrs.NewSensorLogger("./k_state.csv",
		"T", "U1", "U2", "U3", "Z1", "Z2", "Z3", "Phi", "Theta", "Psi", "H1", "H2", "H3", "N1", "N2", "N3",
		"V1", "V2", "V3", "C1", "C2", "C3", "Phi0", "Theta0", "Psi0", "D1", "D2", "D3", "L1", "L2", "L3")
	lKalman := ahrs.NewSensorLogger("./k_kalman.csv",
		"T", "U1", "U2", "U3", "Z1", "Z2", "Z3", "Phi", "Theta", "Psi", "H1", "H2", "H3", "N1", "N2", "N3",
		"V1", "V2", "V3", "C1", "C2", "C3", "Phi0", "Theta0", "Psi0", "D1", "D2", "D3", "L1", "L2", "L3")
	lPredict := ahrs.NewSensorLogger("./k_predict.csv",
		"T", "U1", "U2", "U3", "Z1", "Z2", "Z3", "Phi", "Theta", "Psi", "H1", "H2", "H3", "N1", "N2", "N3",
		"V1", "V2", "V3", "C1", "C2", "C3", "Phi0", "Theta0", "Psi0", "D1", "D2", "D3", "L1", "L2", "L3")
	lVar := ahrs.NewSensorLogger("./k_var.csv",
		"T", "U1", "U2", "U3", "Z1", "Z2", "Z3", "Phi", "Theta", "Psi", "H1", "H2", "H3", "N1", "N2", "N3",
		"V1", "V2", "V3", "C1", "C2", "C3", "Phi0", "Theta0", "Psi0", "D1", "D2", "D3", "L1", "L2", "L3")
	// U, W, A, B, M
	lMeas := ahrs.NewSensorLogger("./k_meas.csv",
		"T", "U1", "U2", "U3", "W1", "W2", "W3", "A1", "A2", "A3", "B1", "B2", "B3", "M1", "M2", "M3")
	lPMeas := ahrs.NewSensorLogger("./k_predmeas.csv",
		"T", "U1", "U2", "U3", "W1", "W2", "W3", "A1", "A2", "A3", "B1", "B2", "B3", "M1", "M2", "M3")
	lKMeas := ahrs.NewSensorLogger("./k_kalmeas.csv",
		"T", "U1", "U2", "U3", "W1", "W2", "W3", "A1", "A2", "A3", "B1", "B2", "B3", "M1", "M2", "M3")

	switch scenario {
	case "takeoff":
		sit = sitTakeoffDef
	case "turn":
		sit = sitTurnDef
	default:
		log.Printf("Loading data from %s\n", scenario)
		sit, err = NewSituationFromFile(scenario)
		if err != nil {
			log.Fatalln(err.Error())
		}
	}

	// This is where it all happens
	fmt.Println("Running Simulation")
	s0 := new(ahrs.State)       // Actual state from simulation, for comparison
	m  := ahrs.NewMeasurement()
	pm := ahrs.NewMeasurement()
	t := sit.BeginTime()
	tNextUpdate := t + udt
	sit.Measurement(t, m, !asiInop, !gpsInop, true, !magInop,
		asiNoise, gpsNoise, accelNoise, gyroNoise, magNoise,
		uBias, accelBias, gyroBias, magBias)
	s := ahrs.Initialize(m)
	// These next few just for testing with a correct starting state
	//TODO testing
	/*
	sit.Interpolate(t, s0, accelBias, gyroBias, magBias)
	// U, Z, E, H, N,
	// V, C, F, D, L
	s.U1 = s0.U1
	s.U2 = s0.U2
	s.U3 = s0.U3
	s.Z1 = s0.Z1
	s.Z2 = s0.Z2
	s.Z3 = s0.Z3
	s.E0 = s0.E0
	s.E1 = s0.E1
	s.E2 = s0.E2
	s.E3 = s0.E3
	s.H1 = s0.H1
	s.H2 = s0.H2
	s.H3 = s0.H3
	s.N1 = s0.N1
	s.N2 = s0.N2
	s.N3 = s0.N3
	s.V1 = s0.V1
	s.V2 = s0.V2
	s.V3 = s0.V3
	s.C1 = s0.C1
	s.C2 = s0.C2
	s.C3 = s0.C3
	s.F0 = s0.F0
	s.F1 = s0.F1
	s.F2 = s0.F2
	s.F3 = s0.F3
	s.D1 = s0.D1
	s.D2 = s0.D2
	s.D3 = s0.D3
	s.L1 = s0.L1
	s.L2 = s0.L2
	s.L3 = s0.L3
	// Done for testing
	*/
	for {
		if t>tNextUpdate-1e-9 {
			t = tNextUpdate
		}

		// Peek behind the curtain: the "actual" state, which the algorithm doesn't know
		if err := sit.Interpolate(t, s0, accelBias, gyroBias, magBias); err != nil {
			break
		}
		phi, theta, psi := ahrs.FromQuaternion(s0.E0, s0.E1, s0.E2, s0.E3)
		phi0, theta0, psi0 := ahrs.FromQuaternion(s0.F0, s0.F1, s0.F2, s0.F3)
		lActual.Log(s0.T,
			s0.U1, s0.U2, s0.U3,
			s0.Z1, s0.Z2, s0.Z3,
			phi / Deg, theta / Deg, psi / Deg,
			s0.H1, s0.H2, s0.H3,
			s0.N1, s0.N2, s0.N3,
			s0.V1, s0.V2, s0.V3,
			s0.C1, s0.C2, s0.C3,
			phi0 / Deg, theta0 / Deg, psi0 / Deg,
			s0.D1, s0.D2, s0.D3,
			s0.L1, s0.L2, s0.L3,
		)

		// Take sensor measurements
		if err := sit.Measurement(t, m, !asiInop, !gpsInop, true, !magInop,
			asiNoise, gpsNoise, accelNoise, gyroNoise, magNoise,
			uBias, accelBias, gyroBias, magBias); err != nil {
			break
		}
		lMeas.Log(m.T,
			m.U1, m.U2, m.U3,
			m.W1, m.W2, m.W3,
			m.A1, m.A2, m.A3,
			m.B1, m.B2, m.B3,
			m.M1, m.M2, m.M3,
		)

		// Predict stage of Kalman filter
		s.Predict(t)
		phi, theta, psi = ahrs.FromQuaternion(s.E0, s.E1, s.E2, s.E3)
		phi0, theta0, psi0 = ahrs.FromQuaternion(s.F0, s.F1, s.F2, s.F3)
		lPredict.Log(s.T,
			s.U1, s.U2, s.U3,
			s.Z1, s.Z2, s.Z3,
			phi / Deg, theta / Deg, psi / Deg,
			s.H1, s.H2, s.H3,
			s.N1, s.N2, s.N3,
			s.V1, s.V2, s.V3,
			s.C1, s.C2, s.C3,
			phi0 / Deg, theta0 / Deg, psi0 / Deg,
			s.D1, s.D2, s.D3,
			s.L1, s.L2, s.L3,
		)

		pm = s.PredictMeasurement()
		lPMeas.Log(pm.T,
			pm.U1, pm.U2, pm.U3,
			pm.W1, pm.W2, pm.W3,
			pm.A1, pm.A2, pm.A3,
			pm.B1, pm.B2, pm.B3,
			pm.M1, pm.M2, pm.M3,
		)

		// Update stage of Kalman filter
		if t > tNextUpdate - 1e-9 {
			tNextUpdate += udt
			s.Update(m)
		}

		pm = s.PredictMeasurement()
		lKMeas.Log(pm.T,
			pm.U1, pm.U2, pm.U3,
			pm.W1, pm.W2, pm.W3,
			pm.A1, pm.A2, pm.A3,
			pm.B1, pm.B2, pm.B3,
			pm.M1, pm.M2, pm.M3,
		)

		phi, theta, psi = ahrs.FromQuaternion(s.E0, s.E1, s.E2, s.E3)
		phi0, theta0, psi0 = ahrs.FromQuaternion(s.F0, s.F1, s.F2, s.F3)
		dphi, dtheta, dpsi := ahrs.VarFromQuaternion(s.E0, s.E1, s.E2, s.E3,
			math.Sqrt(s.M.Get(6, 6)), math.Sqrt(s.M.Get(7, 7)),
			math.Sqrt(s.M.Get(8, 8)), math.Sqrt(s.M.Get(9, 9)))
		dphi0, dtheta0, dpsi0 := ahrs.VarFromQuaternion(s.F0, s.F1, s.F2, s.F3,
			math.Sqrt(s.M.Get(22, 22)), math.Sqrt(s.M.Get(23, 23)),
			math.Sqrt(s.M.Get(24, 24)), math.Sqrt(s.M.Get(25, 25)))
		lKalman.Log(s.T,
			s.U1, s.U2, s.U3,
			s.Z1, s.Z2, s.Z3,
			phi / Deg, theta / Deg, psi / Deg,
			s.H1, s.H2, s.H3,
			s.N1, s.N2, s.N3,
			s.V1, s.V2, s.V3,
			s.C1, s.C2, s.C3,
			phi0 / Deg, theta0 / Deg, psi0 / Deg,
			s.D1, s.D2, s.D3,
			s.L1, s.L2, s.L3,
		)
		lVar.Log(s.T,
			math.Sqrt(s.M.Get(0, 0)), math.Sqrt(s.M.Get(1, 1)), math.Sqrt(s.M.Get(2, 2)),
			math.Sqrt(s.M.Get(3, 3)), math.Sqrt(s.M.Get(4, 4)), math.Sqrt(s.M.Get(5, 5)),
			dphi / Deg, dtheta / Deg, dpsi / Deg,
			math.Sqrt(s.M.Get(10, 10)), math.Sqrt(s.M.Get(11, 11)), math.Sqrt(s.M.Get(12, 12)),
			math.Sqrt(s.M.Get(13, 13)), math.Sqrt(s.M.Get(14, 14)), math.Sqrt(s.M.Get(15, 15)),
			math.Sqrt(s.M.Get(16, 16)), math.Sqrt(s.M.Get(17, 17)), math.Sqrt(s.M.Get(18, 18)),
			math.Sqrt(s.M.Get(19, 19)), math.Sqrt(s.M.Get(20, 20)), math.Sqrt(s.M.Get(21, 21)),
			dphi0 / Deg, dtheta0 / Deg, dpsi0 / Deg,
			math.Sqrt(s.M.Get(26, 26)), math.Sqrt(s.M.Get(27, 27)), math.Sqrt(s.M.Get(28, 28)),
			math.Sqrt(s.M.Get(29, 29)), math.Sqrt(s.M.Get(30, 30)), math.Sqrt(s.M.Get(31, 31)),
		)

		t += pdt
		if s.U1 < 0 {
			s = ahrs.Initialize(m)
		}
	}

	// Clean up
	lActual.Close()
	lKalman.Close()
	lPredict.Close()
	lPMeas.Close()
	lVar.Close()
	lMeas.Close()
	lKMeas.Close()

	// Run analysis web server
	fmt.Println("Serving charts")
	http.Handle("/", http.FileServer(http.Dir("./")))
	http.ListenAndServe(":8080", nil)
}
