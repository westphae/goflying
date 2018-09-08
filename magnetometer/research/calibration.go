package main

import (
	"encoding/csv"
	"encoding/json"
	"flag"
	"fmt"
	"html/template"
	"io"
	"io/ioutil"
	"log"
	"math"
	"math/rand"
	"net/http"
	"os"
	"path/filepath"
	"strconv"
	"sync"
	"time"

	".."
	"../../ahrs"
	"../../mpu9250"
	"github.com/gorilla/websocket"
)

const (
	numMPURetries = 5     // Number of retries for connecting to MPU
	freqDefault   = 4     // Polling frequency
	M0            = 56000 // Some default Earth magnetic field
)

var upgrader = websocket.Upgrader{
	ReadBufferSize:  1024,
	WriteBufferSize: 1024,
	CheckOrigin:     func(r *http.Request) bool { return true },
}

var k, l [3]float64

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
	var (
		freq           float64
		startTM, endTM float64
		usage          string
		reqData        chan chan map[string]interface{} // A chan over which we send a chan to receive data
	)

	// Which kind of system to run: real (default) or random or replay?
	// calibrate hw [-f | --freq freq]
	// calibrate rand [-f } --freq freq]
	// calibrate replay [-s | --start start_time] [-e | --end end_time]
	hwCmd := flag.NewFlagSet("hw", flag.ExitOnError)
	usage = "Frequency to read hardware at, Hz"
	hwCmd.Float64Var(&freq, "freq", freqDefault, usage)
	hwCmd.Float64Var(&freq, "f", freqDefault, usage+" (shorthand)")

	randCmd := flag.NewFlagSet("rand", flag.ExitOnError)
	usage = "Frequency to read hardware at, Hz"
	randCmd.Float64Var(&freq, "freq", freqDefault, usage)
	randCmd.Float64Var(&freq, "f", freqDefault, usage+" (shorthand)")

	replayCmd := flag.NewFlagSet("replay", flag.ExitOnError)
	usage = "Frequency to send data at, Hz"
	replayCmd.Float64Var(&freq, "freq", freqDefault, usage)
	replayCmd.Float64Var(&freq, "f", freqDefault, usage+" (shorthand)")
	usage = "file TM value at which to start simulating, default 0"
	replayCmd.Float64Var(&startTM, "start", 0, usage)
	replayCmd.Float64Var(&startTM, "s", 0, usage)
	usage = "file TM value at which to end simulating, default 0"
	replayCmd.Float64Var(&endTM, "end", 0, usage)
	replayCmd.Float64Var(&endTM, "e", -1, usage)

	if len(os.Args) < 2 {
		fmt.Println("You must enter a command: hw, rand, or replay")
		os.Exit(1)
	}

	if stratuxConf, err := ioutil.ReadFile("/etc/stratux.conf"); err != nil {
		log.Printf("couldn't open stratux.conf: %s", err)
	} else {
		var conf map[string]*json.RawMessage
		if err := json.Unmarshal(stratuxConf, &conf); err != nil {
			log.Printf("error parsing stratux.conf: %s", err)
		} else {
			if byteK, ok := conf["K"]; ok {
				if err = json.Unmarshal(*byteK, &k); err != nil {
					log.Printf("No k in stratux.conf")
				}
			}
			if byteL, ok := conf["L"]; ok {
				if err = json.Unmarshal(*byteL, &l); err != nil {
					log.Printf("No l in stratux.conf")
				}
			}
		}
	}

	switch os.Args[1] {
	case "hw":
		hwCmd.Parse(os.Args[2:])

		mpu, err := openMPU9250()
		if err != nil {
			log.Println(err)
			return
		}
		defer mpu.CloseMPU()
		log.Println("MPU9250 initialized successfully.")

		reqData = readMPUData(mpu.CAvg, time.Duration(1000/freq)*time.Millisecond)
	case "rand":
		randCmd.Parse(os.Args[2:])

		res := [6]float64{
			0.2 * rand.NormFloat64(),   // M1-offset
			1 + 0.2*rand.NormFloat64(), // M1-scaling
			0.2 * rand.NormFloat64(),   // M2-offset
			1 + 0.2*rand.NormFloat64(), // M2-scaling
			0.2 * rand.NormFloat64(),   // M3-offset
			1 + 0.2*rand.NormFloat64(), // M3-scaling
		}
		log.Printf("Sending random data for true mag %v\n", res)

		reqData = readMPUData(genRandomData(res), time.Duration(1000/freq)*time.Millisecond)
	case "replay":
		replayCmd.Parse(os.Args[2:])
		fn := replayCmd.Arg(0)
		csvFile, err := os.Open(fn)
		if err != nil {
			log.Println(err)
			os.Exit(1)
		}
		defer csvFile.Close()
		log.Printf("Sending data from file %s from timestamp %f to %f\n", fn, startTM, endTM)

		reqData = readMPUData(genFileData(csvFile, startTM, endTM), time.Duration(1000/freq)*time.Millisecond)
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

	cM, cMagKal := magkal.NewMagKal(k, l, magkal.ComputeKalman)

	go func() {
		var (
			ch     chan map[string]interface{}
			cur    *mpu9250.MPUData
			logMap = make(map[string]interface{})
			n      magkal.MagKalState
		)

		t0 := time.Now()
		ticker := time.NewTicker(freq)
		for {
			<-ticker.C

			cur = <-data

			// Data processing goes here.
			cM <- *&ahrs.Measurement{T: float64(cur.T.Sub(t0).Nanoseconds()/1000000) / 1000,
				M1: cur.M1, M2: cur.M2, M3: cur.M3}
			n = <-cMagKal
			k = n.K
			l = n.L

			updateLogMap(t0, cur, &n, logMap)
			for len(reqData) > 0 {
				ch = <-reqData
				ch <- logMap
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

			out <- &mpu9250.MPUData{
				T:  time.Now(),
				TM: time.Now(),
				M1: M0 * (magVals[0] + magVals[1]*math.Cos(psi)*math.Cos(theta)),
				M2: M0 * (magVals[2] + magVals[3]*math.Sin(psi)*math.Cos(theta)),
				M3: M0 * (magVals[4] + magVals[5]*math.Sin(theta)),
			}
		}
	}()
	return
}

func genFileData(f io.Reader, start float64, end float64) (out chan *mpu9250.MPUData) {
	out = make(chan *mpu9250.MPUData)

	var (
		err                              error
		colT, colTM, colM1, colM2, colM3 int
		t0, t, tm0, tm, m1, m2, m3       float64
		tn                               time.Time
		foundTM                          bool
	)
	csvReader := csv.NewReader(f)
	header, err := csvReader.Read()
	if err != nil {
		fmt.Println(err)
		os.Exit(1)
	}
	for i, s := range header {
		switch s {
		case "T":
			colT = i
		case "TM":
			colTM = i
			foundTM = true
		case "M1":
			colM1 = i
		case "M2":
			colM2 = i
		case "M3":
			colM3 = i
		}
	}
	if !foundTM {
		colTM = colT
	}

	go func() {
		for {
			r, err := csvReader.Read()
			if err == io.EOF {
				break
			}
			if err != nil {
				log.Fatal(err)
			}

			t, err = strconv.ParseFloat(r[colT], 64)
			if err != nil {
				break
			}
			if (start != 0 && t < start) || (end != 0 && t > end) {
				continue
			}
			if t0 == 0 {
				tn = time.Now()
				t0 = t
			}
			tm, err = strconv.ParseFloat(r[colTM], 64)
			if err != nil {
				break
			}
			if tm0 == 0 {
				tm0 = tm
			}
			m1, err = strconv.ParseFloat(r[colM1], 64)
			if err != nil {
				break
			}
			m2, err = strconv.ParseFloat(r[colM2], 64)
			if err != nil {
				break
			}
			m3, err = strconv.ParseFloat(r[colM3], 64)
			if err != nil {
				break
			}

			log.Printf("T: %f, TM: %f\n", t-t0, tm-tm0)
			out <- &mpu9250.MPUData{
				T:  tn.Add(time.Duration((t-t0)*1000) * time.Millisecond),
				TM: tn.Add(time.Duration((tm-tm0)*1000) * time.Millisecond),
				M1: m1,
				M2: m2,
				M3: m3,
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
		reqData <- myData
		if err = conn.WriteJSON(<-myData); err != nil {
			log.Printf("Error writing to websocket: %s\n", err)
			break
		}
	}
	log.Println("Closing client")
}

var (
	m1Min = ahrs.Big
	m1Max = -ahrs.Big
	m2Min = ahrs.Big
	m2Max = -ahrs.Big
	m3Min = ahrs.Big
	m3Max = -ahrs.Big
)

func updateLogMap(t0 time.Time, m *mpu9250.MPUData, n *magkal.MagKalState, p map[string]interface{}) {
	m1Min, m1Max = math.Min(m1Min, m.M1), math.Max(m1Max, m.M1)
	m2Min, m2Max = math.Min(m2Min, m.M2), math.Max(m2Max, m.M2)
	m3Min, m3Max = math.Min(m3Min, m.M3), math.Max(m3Max, m.M3)

	var sensorLogMap = map[string]func(t0 time.Time, m *mpu9250.MPUData, n *magkal.MagKalState) float64{
		"T": func(t0 time.Time, m *mpu9250.MPUData, n *magkal.MagKalState) float64 {
			return float64(m.T.Sub(t0).Nanoseconds()/1000000) / 1000
		},
		"TM": func(t0 time.Time, m *mpu9250.MPUData, n *magkal.MagKalState) float64 {
			return float64(m.TM.Sub(t0).Nanoseconds()/1000000) / 1000
		},
		"M1":    func(t0 time.Time, m *mpu9250.MPUData, n *magkal.MagKalState) float64 { return m.M1 },
		"M2":    func(t0 time.Time, m *mpu9250.MPUData, n *magkal.MagKalState) float64 { return m.M2 },
		"M3":    func(t0 time.Time, m *mpu9250.MPUData, n *magkal.MagKalState) float64 { return m.M3 },
		"MMin1": func(t0 time.Time, m *mpu9250.MPUData, n *magkal.MagKalState) float64 { return m1Min },
		"MMax1": func(t0 time.Time, m *mpu9250.MPUData, n *magkal.MagKalState) float64 { return m1Max },
		"MMin2": func(t0 time.Time, m *mpu9250.MPUData, n *magkal.MagKalState) float64 { return m2Min },
		"MMax2": func(t0 time.Time, m *mpu9250.MPUData, n *magkal.MagKalState) float64 { return m2Max },
		"MMin3": func(t0 time.Time, m *mpu9250.MPUData, n *magkal.MagKalState) float64 { return m3Min },
		"MMax3": func(t0 time.Time, m *mpu9250.MPUData, n *magkal.MagKalState) float64 { return m3Max },
		"K1":    func(t0 time.Time, m *mpu9250.MPUData, n *magkal.MagKalState) float64 { return n.K[0] },
		"K2":    func(t0 time.Time, m *mpu9250.MPUData, n *magkal.MagKalState) float64 { return n.K[1] },
		"K3":    func(t0 time.Time, m *mpu9250.MPUData, n *magkal.MagKalState) float64 { return n.K[2] },
		"L1":    func(t0 time.Time, m *mpu9250.MPUData, n *magkal.MagKalState) float64 { return n.L[0] },
		"L2":    func(t0 time.Time, m *mpu9250.MPUData, n *magkal.MagKalState) float64 { return n.L[1] },
		"L3":    func(t0 time.Time, m *mpu9250.MPUData, n *magkal.MagKalState) float64 { return n.L[2] },
		"MM1":   func(t0 time.Time, m *mpu9250.MPUData, n *magkal.MagKalState) float64 { return n.K[0]*m.M1 + n.L[0] },
		"MM2":   func(t0 time.Time, m *mpu9250.MPUData, n *magkal.MagKalState) float64 { return n.K[1]*m.M2 + n.L[1] },
		"MM3":   func(t0 time.Time, m *mpu9250.MPUData, n *magkal.MagKalState) float64 { return n.K[2]*m.M3 + n.L[2] },
	}

	for k := range sensorLogMap {
		p[k] = sensorLogMap[k](t0, m, n)
	}
}
