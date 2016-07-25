package main

import (
	"fmt"
	"log"
	"os"
	"strings"
)

type AHRSLogger struct {
	f	*os.File
	h	[]string
	fmt 	string
}

func NewAHRSLogger(fn string, h ...string) (l AHRSLogger) {
	l.h = h
	f, err := os.Create(fn)
	l.f = f
	if err != nil {
		log.Fatalln(err)
	}

	fmt.Fprint(l.f, strings.Join(l.h, ","), "\n")
	s := strings.Repeat("%f,", len(l.h))
	l.fmt = strings.Join([]string{s[:len(s)-1], "\n"}, "")
	return
}

func (l *AHRSLogger) Log(v ...interface{}) {
	fmt.Fprintf(l.f, l.fmt, v...)
}

func (l *AHRSLogger) Close() {
	l.f.Close()
}

