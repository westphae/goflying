/*
Client-Server package adapted from Mat Ryer's Go Blueprints examples
see https://github.com/matryer/goblueprints
This book is highly recommended!
*/

package main

import (
	"flag"
	"html/template"
	"log"
	"net/http"
	"path/filepath"
	"runtime"
	"sync"
)

// templ represents a single template
type templateHandler struct {
	once     sync.Once
	filename string
	templ    *template.Template
}

// ServeHTTP handles the HTTP request.
func (t *templateHandler) ServeHTTP(w http.ResponseWriter, r *http.Request) {
	t.once.Do(func() {
		t.templ = template.Must(template.ParseFiles(filepath.Join("templates", t.filename)))
	})
	t.templ.Execute(w, r)
}

type DataListener interface {
	SetRoom(*room)
	Init()
	Run()
	GetData() *AHRSData
	Close()
}

type AHRSData struct {
	// Kalman state variables
	U1, U2, U3    	float64             // Vector for airspeed, aircraft frame, kt
	Z1, Z2, Z3      float64             // Vector for rate of change of airspeed, aircraft frame, G
	E0, E1, E2, E3	float64             // Quaternion rotating earth frame to aircraft frame
	H1, H2, H3      float64             // Vector for gyro rates, earth frame, °/s
	N1, N2, N3    	float64             // Vector for earth's magnetic field, earth (inertial) frame, µT

	V1, V2, V3    	float64             // (Bias) Vector for windspeed, earth frame, kt
	C1, C2, C3      float64             // Bias vector for accelerometer, sensor frame, G
	F0, F1, F2, F3	float64             // (Bias) quaternion rotating sensor frame to aircraft frame
	D1, D2, D3      float64             // Bias vector for gyro rates, sensor frame, °/s
	L1, L2, L3      float64             // Bias vector for magnetometer direction, sensor frame, µT

	// Kalman state uncertainties
	DU1, DU2, DU3    	float64     // Vector for airspeed, aircraft frame, kt
	DZ1, DZ2, DZ3           float64     // Vector for rate of change of airspeed, aircraft frame, G
	DE0, DE1, DE2, DE3	float64     // Quaternion rotating earth frame to aircraft frame
	DH1, DH2, DH3           float64     // Vector for gyro rates, earth frame, °/s
	DN1, DN2, DN3    	float64     // Vector for earth's magnetic field, earth (inertial) frame, µT

	DV1, DV2, DV3    	float64     // (Bias) Vector for windspeed, earth frame, kt
	DC1, DC2, DC3           float64     // Bias vector for accelerometer, sensor frame, G
	DF0, DF1, DF2, DF3	float64     // (Bias) quaternion rotating sensor frame to aircraft frame
	DD1, DD2, DD3           float64     // Bias vector for gyro rates, sensor frame, °/s
	DL1, DL2, DL3           float64     // Bias vector for magnetometer direction, sensor frame, µT

	// Measurement variables
	UValid, WValid, SValid, MValid bool    // Do we have valid airspeed, GPS, accel/gyro, and magnetometer readings?
	S1, S2, S3                     float64 // Vector of airspeeds
	W1, W2, W3                     float64 // Vector of GPS speed in N/S, E/W and U/D directions, kt, latlong axes, earth (inertial) frame
	A1, A2, A3                     float64 // Vector holding accelerometer readings, G, aircraft (accelerated) frame
	B1, B2, B3                     float64 // Vector of gyro rates in roll, pitch, heading axes, °/s, aircraft (accelerated) frame
	M1, M2, M3                     float64 // Vector of magnetometer readings, µT, aircraft (accelerated) frame
	T                              float64 // Timestamp of GPS, airspeed and magnetometer readings

	// Final output
	Pitch, Roll, Heading    float64
}

func main() {
	var addr = flag.String("addr", ":8080", "The port for the AHRS data publication.")
	var src = flag.String("src", "mpu9250", "Source of ahrs data: rand, sim or mpu9250.")
	flag.Parse() // parse the flags

	// get the room going
	r := newRoom()
	go r.run()

	// get the MPU Listener going
	var ml DataListener
	switch *src {
	case "rand":
		ml = new(RandListener)
	case "sim":
		// ml = new(SimListener)
	case "mpu9250":
		if runtime.GOARCH != "arm" {
			log.Println("--src can only be mpu on arm architecture")
			return
		}
		ml = new(MPU9250Listener)
	default:
		log.Println("--src must be rand, sim or mpu9250 (default)")
		return
	}

	ml.SetRoom(r)
	ml.Init()
	defer ml.Close()
	go ml.Run()
	log.Println("MPU listener started")

	// start the web server
	http.Handle("/", &templateHandler{filename: "messages.html"})
	// serve web client files
	http.HandleFunc("/d3.min.js", func(w http.ResponseWriter, r *http.Request) { http.ServeFile(w, r, "js/d3.min.js") })
	http.Handle("/room", r)
	log.Println("Starting web server on", *addr)
	if err := http.ListenAndServe(*addr, nil); err != nil {
		log.Fatal("ListenAndServe:", err)
	}
}
