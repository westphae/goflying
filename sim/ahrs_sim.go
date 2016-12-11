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
	"encoding/json"
)

type config struct{
	StateVars []string // List of state variables to output
	OtherVars []string // List of extra variables to output
	CMVars    []string // List of control/measurement variables to output
}

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
		kconfig                                                 config
		err 							error
		st							map[string]float64
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
	pm := ahrs.NewMeasurement() // Predicted Measurement

	fmt.Println("Simulation parameters:")
	switch strings.ToLower(algo) {
	case "simple":
		fmt.Println("Running simple AHRS")
		kconfig = config{
			StateVars: []string{"Phi", "Theta", "Psi"},
			OtherVars: []string{"rollGPS","pitchGPS","headingGPS","TR","GS","TR"},
			CMVars: []string{"W1","W2","W3","A1","A2","A3","B1","B2","B3","M1","M2","M3"},
		}
		s = ahrs.InitializeSimple(m)
	case "heuristic":
		fmt.Println("Running heuristic AHRS")
		kconfig = config{}
		s = ahrs.InitializeHeuristic(m)
	case "kalman":
		fmt.Println("Running Kalman AHRS")
		kconfig = config{
			StateVars: []string{"U1","U2","U3","Z1","Z2","Z3","Phi","Theta","Psi","H1","H2","H3","N1","N2",
				"N3","V1","V2","V3","C1","C2","C3","Phi0","Theta0","Psi0","D1","D2","D3","L1","L2","L3"},
			CMVars: []string{"U1","U2","U3","W1","W2","W3","A1","A2","A3","B1","B2","B3","M1","M2","M3"},
		}
		s = ahrs.InitializeKalman(m)
	case "kalman1":
		fmt.Println("Running Kalman1 AHRS")
		kconfig = config{
			StateVars: []string{"Phi","Theta","Psi","H1","H2","H3","Phi0","Theta0","Psi0","D1","D2","D3"},
			CMVars: []string{"A1","A2","A3","B1","B2","B3","M1","M2","M3"},
		}
		s = ahrs.InitializeKalman1(m)
	case "kalman2":
		fmt.Println("Running Kalman2 AHRS")
		kconfig = config{
			StateVars: []string{"U1","U2","U3","Z1","Z2","Z3","Phi","Theta","Psi",
				"H1","H2","H3","C1","C2","C3","Phi0","Theta0","Psi0","D1","D2","D3"},
			CMVars: []string{"A1","A2","A3","B1","B2","B3","M1","M2","M3"},
		}
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
	lKOther := ahrs.NewSensorLogger("./k_other.csv", append([]string{"T"}, kconfig.OtherVars...)...)

	// This is where it all happens
	fmt.Println("Running Simulation")
	t := sit.BeginTime()
	tNextUpdate := t + udt
	sit.Measurement(t, m, !asiInop, !gpsInop, true, !magInop,
		asiNoise, gpsNoise, accelNoise, gyroNoise, magNoise,
		uBias, accelBias, gyroBias, magBias)
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
			log.Printf("Interpolation error at time %f: %s\n", t, err)
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
			log.Printf("Measurement error at time %f: %s\n", t, err)
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
		st = *s.GetStateMap()
		lPredict.Log(st["T"],
			st["U1"], st["U2"], st["U3"],
			st["Z1"], st["Z2"], st["Z3"],
			st["Phi"], st["Theta"], st["Psi"],
			st["H1"], st["H2"], st["H3"],
			st["N1"], st["N2"], st["N3"],
			st["V1"], st["V2"], st["V3"],
			st["C1"], st["C2"], st["C3"],
			st["Phi0"], st["Theta0"], st["Psi0"],
			st["D1"], st["D2"], st["D3"],
			st["L1"], st["L2"], st["L3"],
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
			log.Printf("Time: %.2f\n", t)
		}

		pm = s.PredictMeasurement()
		lKMeas.Log(pm.T,
			pm.U1, pm.U2, pm.U3,
			pm.W1, pm.W2, pm.W3,
			pm.A1, pm.A2, pm.A3,
			pm.B1, pm.B2, pm.B3,
			pm.M1, pm.M2, pm.M3,
		)

		st = *s.GetStateMap()
		lKalman.Log(st["T"],
			st["U1"], st["U2"], st["U3"],
			st["Z1"], st["Z2"], st["Z3"],
			st["Phi"], st["Theta"], st["Psi"],
			st["H1"], st["H2"], st["H3"],
			st["N1"], st["N2"], st["N3"],
			st["V1"], st["V2"], st["V3"],
			st["C1"], st["C2"], st["C3"],
			st["Phi0"], st["Theta0"], st["Psi0"],
			st["D1"], st["D2"], st["D3"],
			st["L1"], st["L2"], st["L3"],
		)
		lVar.Log(st["T"],
			st["T"], st["dU1"], st["dU2"], st["dU3"], st["dZ1"], st["dZ2"], st["dZ3"],
			st["dPhi"], st["dTheta"], st["dPsi"], st["dH1"], st["dH2"], st["dH3"],
			st["dN1"], st["dN2"], st["dN3"], st["dV1"], st["dV2"], st["dV3"],
			st["dC1"], st["dC2"], st["dC3"], st["dPhi0"], st["dTheta0"], st["dPsi0"],
			st["dD1"], st["dD2"], st["dD3"], st["dL1"], st["dL2"], st["dL3"],
		)

		vals := func() (v []float64) {
			v = make([]float64, len(kconfig.OtherVars) + 1)
			v[0] = st["T"]
			for i, k := range kconfig.OtherVars {
				v[i+1] = st[k]
				if (strings.HasPrefix(k, "roll") || strings.HasPrefix(k, "pitch") ||
					strings.HasPrefix(k, "heading") || strings.HasPrefix(k, "TR")) {
					v[i+1] /= Deg
				}
			}
			return
		}()
		lKOther.Log(vals...)

		t += pdt
	}

	// Clean up
	lActual.Close()
	lKalman.Close()
	lPredict.Close()
	lPMeas.Close()
	lVar.Close()
	lMeas.Close()
	lKMeas.Close()
	lKOther.Close()
	js, err := json.Marshal(&kconfig)
	ioutil.WriteFile("k_config.json", js, 0644)

	// Run analysis web server
	fmt.Println("Serving charts")
	http.Handle("/", http.FileServer(http.Dir("./")))
	http.ListenAndServe(":8080", nil)
}
