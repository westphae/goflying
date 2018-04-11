/*
Client-Server package adapted from Mat Ryer's Go Blueprints examples
see https://github.com/matryer/goblueprints
This book is highly recommended!
*/

package main

import (
	"flag"
	"fmt"
	"html/template"
	"log"
	"net/http"
	"path/filepath"
	"sync"

	"github.com/westphae/goflying/ahrsweb"
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
		t.templ = template.Must(template.ParseFiles(filepath.Join("res", t.filename)))
	})
	t.templ.Execute(w, r)
}

func main() {
	var addr = flag.String("addr", fmt.Sprintf(":%d", ahrsweb.Port), "The port for the AHRS data publication.")
	flag.Parse() // parse the flags

	// get the room going
	r := ahrsweb.NewRoom()
	go r.Run()

	// start the web server
	http.Handle("/", &templateHandler{filename: "analyzer.html"})
	http.Handle("/magnetometer", &templateHandler{filename: "magnetometer.html"})
	http.HandleFunc("/d3.min.js",
		func(w http.ResponseWriter, r *http.Request) { http.ServeFile(w, r, "res/d3.min.js") })
	http.HandleFunc("/magcal.js",
		func(w http.ResponseWriter, r *http.Request) { http.ServeFile(w, r, "res/magcal.js") })
	http.Handle("/ahrsweb", r)
	log.Println("AHRSWeb: Starting web server on", *addr)
	if err := http.ListenAndServe(*addr, nil); err != nil {
		log.Fatal("AHRSWeb: ListenAndServe fatal error:", err.Error())
	}
}
