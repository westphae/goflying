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

func main() {
	var addr = flag.String("addr", ":8080", "The addr of the application.")
	flag.Parse() // parse the flags
	r := newRoom()
	http.Handle("/", &templateHandler{filename: "messages.html"})
	http.HandleFunc("/d3.min.js", func(w http.ResponseWriter, r *http.Request) { http.ServeFile(w, r, "js/d3.min.js") })
	http.Handle("/room", r)
	log.Println("Web handler started, running room & listener")
	// get the room going
	go r.run()
	log.Println("Room started, starting new MPU listener")
	// get the MPU Listener going
	ml := NewMPUListener(r)
	log.Println("MPU listener created")
	go ml.run()
	log.Println("MPU listener started")
	// start the web server
	log.Println("Starting web server on", *addr)
	if err := http.ListenAndServe(*addr, nil); err != nil {
		log.Fatal("ListenAndServe:", err)
	}
}
