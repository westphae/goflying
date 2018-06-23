package main

import (
	"flag"
	"fmt"
	"html/template"
	"io"
	"io/ioutil"
	"log"
	"math"
	"math/rand"
	"net/http"
	"path/filepath"
	"sync"
	"time"

	"github.com/gorilla/websocket"
	"github.com/westphae/goflying/ahrs"
	"github.com/westphae/goflying/mpu9250"
)

const (
	numMPURetries = 5 // Number of retries for connecting to MPU
	freq = 250 * time.Millisecond // Polling frequency
	M0 = 56000 // Some default Earth magnetic field
)

var upgrader = websocket.Upgrader{
	ReadBufferSize:  1024,
	WriteBufferSize: 1024,
	CheckOrigin:     func(r *http.Request) bool { return true },
}

// templ represents a single template
type templateHandler struct {
	once     sync.Once
	filename string
	templ    *template.Template
}

// ServeHTTP handles the HTTP request.
func (t *templateHandler) ServeHTTP(w http.ResponseWriter, r *http.Request) {
	t.once.Do(func() {
		t.templ = template.Must(template.ParseFiles(filepath.Join("res", t.filename)))
	})
	t.templ.Execute(w, r)
}

func main() {
	var reqData chan chan map[string]interface{} // A chan over which we send a chan to receive data

	// Which kind of system to run: real (default) or random?
	flag.Parse()
	log.Println(flag.Arg(0))
	switch flag.Arg(0) {
	case "rand":
		res := [6]float64{
			0.2*rand.NormFloat64(), // M1-offset
			1+0.2*rand.NormFloat64(), // M1-scaling
			0.2*rand.NormFloat64(), // M2-offset
			1+0.2*rand.NormFloat64(), // M2-scaling
			0.2*rand.NormFloat64(), // M3-offset
			1+0.2*rand.NormFloat64(), // M3-scaling
		}
		log.Printf("Sending random data for true mag %v\n", res)
		reqData = readMPUData(genRandomData(res), freq)
	default:
		mpu, err := openMPU9250()
		if err != nil {
			log.Println(err)
			return
		}
		defer mpu.CloseMPU()
		log.Println("MPU9250 initialized successfully.")

		reqData = readMPUData(mpu.CAvg, freq)
	}

	http.Handle("/", &templateHandler{filename: "index.html"})
	http.HandleFunc("/d3.min.js",
		func(w http.ResponseWriter, r *http.Request) { http.ServeFile(w, r, "res/d3.min.js") })
	http.HandleFunc("/magkal.js",
		func(w http.ResponseWriter, r *http.Request) { http.ServeFile(w, r, "res/magkal.js") })
	http.HandleFunc("/websocket", func(w http.ResponseWriter, r *http.Request) { sendData(w, r, reqData) })
	log.Println("Listening for websocket connections on port 8000")
	log.Fatal(http.ListenAndServe(":8000", nil))
}

func openMPU9250() (mpu *mpu9250.MPU9250, err error) {
	for i := 0; i < numMPURetries && mpu == nil; i++ {
		mpu, err = mpu9250.NewMPU9250(250, 4, 50, true, false)
		if err != nil {
			log.Printf("Couldn't initialize MPU9250, attempt %d of %d: %v\n", i, numMPURetries, err)
			time.Sleep(100 * time.Millisecond)
		}
	}
	if err != nil {
		return nil, fmt.Errorf("error connecting to MPU9250: %v", err)
	}
	return mpu, nil
}

func readMPUData(data <-chan *mpu9250.MPUData, freq time.Duration) (reqData chan chan map[string]interface{}) {
	reqData = make(chan chan map[string]interface{}, 128)

	go func() {
		var (
			ch chan map[string]interface{}
			cur    *mpu9250.MPUData
			logMap = make(map[string]interface{})
		)

		t0 := time.Now()
		ticker := time.NewTicker(freq)
		for {
			<-ticker.C

			cur = <-data

			// Data processing goes here.

			updateLogMap(t0, cur, logMap)
			for len(reqData) > 0 {
				ch = <-reqData
				ch<- logMap
			}
		}
	}()
	return
}

func genRandomData(magVals [6]float64) (out chan *mpu9250.MPUData) {
	out = make(chan *mpu9250.MPUData)

	go func() {
		var (
			psi, theta float64
		)

		for {
			psi = 2 * ahrs.Pi * rand.Float64()
			theta = ahrs.Pi * rand.Float64()

			out<- &mpu9250.MPUData{
				T: time.Now(),
				TM: time.Now(),
				M1: M0 * (magVals[0] + magVals[1]*math.Cos(psi)*math.Cos(theta)),
				M2: M0 * (magVals[2] + magVals[3]*math.Sin(psi)*math.Cos(theta)),
				M3: M0 * (magVals[4] + magVals[5]*math.Sin(theta)),
			}
		}
	}()
	return
}

func sendData(w http.ResponseWriter, r *http.Request, reqData chan chan map[string]interface{}) {
	conn, err := upgrader.Upgrade(w, r, nil)
	if err != nil {
		log.Printf("Error upgrading to websocket: %s\n", err)
		return
	}
	defer conn.Close()
	log.Printf("Client %d opened a connection", len(reqData))

	// Handle ping-pong
	var timeout *time.Timer
	go func() {
		pingTime := time.NewTicker(5 * time.Second)
		for {
			if err = conn.WriteControl(websocket.PingMessage, []byte("ping"), time.Now().Add(10*time.Second)); err != nil {
				log.Printf("ws error sending ping: %s\n", err)
				break
			}
			timeout = time.NewTimer(4 * time.Second)

			select {
			case <-pingTime.C:
				timeout.Stop()
			case <-timeout.C:
				log.Println("ping timeout")
				pingTime.Stop()
				conn.Close()
				break
			}
		}
		log.Println("Stopping ping")
	}()
	conn.SetPongHandler(func(appData string) error {
		timeout.Stop()
		return nil
	})

	// Set up goroutine to read messages from the client, necessary to receive control messages
	go func() {
		var (
			mType int
			r     io.Reader
			s     []byte
			err   error
		)

		log.Println("Listening for messages from a new client")
		for {
			mType, r, err = conn.NextReader()
			if err != nil {
				log.Printf("Error reading from websocket: %s\n", err)
				break
			}
			s, err = ioutil.ReadAll(r)
			log.Printf("Unknown message (type %d) received: %s\n", mType, s)
		}
	}()

	myData := make(chan map[string]interface{})
	for {
		reqData<- myData
		if err = conn.WriteJSON(<-myData); err != nil {
			log.Printf("Error writing to websocket: %s\n", err)
			break
		}
	}
	log.Println("Closing client")
}

func updateLogMap(t0 time.Time, m *mpu9250.MPUData, p map[string]interface{}) {
	var sensorLogMap = map[string]func(t0 time.Time, m *mpu9250.MPUData) float64{
		"T": func(t0 time.Time, m *mpu9250.MPUData) float64 {
			return float64(m.T.Sub(t0).Nanoseconds()/1000000) / 1000
		},
		"TM": func(t0 time.Time, m *mpu9250.MPUData) float64 {
			return float64(m.TM.Sub(t0).Nanoseconds()/1000000) / 1000
		},
		"M1": func(t0 time.Time, m *mpu9250.MPUData) float64 { return m.M1 },
		"M2": func(t0 time.Time, m *mpu9250.MPUData) float64 { return m.M2 },
		"M3": func(t0 time.Time, m *mpu9250.MPUData) float64 { return m.M3 },
	}

	for k := range sensorLogMap {
		p[k] = sensorLogMap[k](t0, m)
	}

	p["O1"] = 0
	p["O2"] = 0
	p["O3"] = 0
	p["S1"] = 1
	p["S2"] = 1
	p["S3"] = 1
}
