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
	Pitch, Roll, Heading                           float32
	Gx, Gy, Gz, Ax, Ay, Az                         int16
	Qx, Qy, Qz, Qw                                 int32
	Mx, My, Mz                                     int16
	Ts, Tsm                                        uint32
	X_accel, Y_accel, Z_accel, X_mag, Y_mag, Z_mag int16
}

func main() {
	var addr = flag.String("addr", ":8080", "The port for the AHRS data publication.")
	var src = flag.String("src", "mpu", "Source of ahrs data: rand, sim or mpu.")
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
	case "mpu":
		if runtime.GOARCH != "arm" {
			log.Println("--src can only be mpu on arm architecture")
			return
		}
		ml = new(MPUListener)
	default:
		log.Println("--src must be rand, sim or mpu (default)")
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
