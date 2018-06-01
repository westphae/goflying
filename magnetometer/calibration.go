package main

import (
	"fmt"
	"html/template"
	"log"
	"net/http"
	"path/filepath"
	"sync"
	"time"

	"github.com/gorilla/websocket"
	"github.com/westphae/goflying/mpu9250"
	"io"
	"io/ioutil"
)

const numMPURetries int = 5

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
	mpu, err := openMPU9250()
	if err != nil {
		log.Println(err)
		return
	}
	defer mpu.CloseMPU()
	log.Println("MPU9250 initialized successfully.")

	reqData := readData(mpu)

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

func readData(mpu *mpu9250.MPU9250) (reqData chan chan map[string]interface{}) {
	var (
		cur    *mpu9250.MPUData
		logMap = make(map[string]interface{})
	)
	reqData = make(chan chan map[string]interface{}, 100)

	go func() {
		var ch chan map[string]interface{}
		t0 := time.Now()
		ticker := time.NewTicker(100 * time.Millisecond)
		for {
			<-ticker.C

			cur = <-mpu.CAvg

			// Data processing goes here.

			updateLogMap(t0, cur, logMap)
			for len(reqData) > 0 {
				ch = <-reqData
				ch <- logMap
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
	go func() {
		pingTime := time.NewTicker(5 * time.Second)
		for {
			<-pingTime.C
			if err = conn.WriteControl(websocket.PingMessage, []byte("ping"), time.Now().Add(10*time.Second)); err != nil {
				log.Printf("ws error sending ping: %s\n", err)
				break
			}
			log.Println("ping sent")
		}
	}()
	conn.SetPongHandler(func(appData string) error {
		log.Println("Pong received")
		return nil
	})

	// Set up goroutine to read control messages from the client
	go func() {
		var (
			mType int
			r     io.Reader
			s     []byte
			err   error
		)

		log.Println("Listening for messages from a client")
		for {
			mType, r, err = conn.NextReader()
			if err != nil {
				log.Println("Error reading from websocket")
				break
			}
			switch mType {
			case websocket.PingMessage:
				log.Println("ping received")
			case websocket.CloseMessage:
				log.Println("close received")
			case websocket.PongMessage:
				log.Println("pong received")
			default:
				s, err = ioutil.ReadAll(r)
				log.Printf("unknown message type %d received: %s\n", mType, s)
			}
		}
	}()

	myData := make(chan map[string]interface{})
	for {
		reqData <- myData
		if err = conn.WriteJSON(<-myData); err != nil {
			log.Printf("Error writing to websocket: %s\n", err)
			break
		}
	}
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
