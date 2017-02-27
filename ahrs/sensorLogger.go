package ahrs

import (
	"fmt"
	"log"
	"os"
	"strings"
)

type SensorLogger struct {
	f   *os.File
	h   []string
	fmt string
}

func NewSensorLogger(fn string, h ...string) (l *SensorLogger) {
	l = new(SensorLogger)
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

func (l *SensorLogger) Log(v ...float64) {
	o := make([]interface{}, len(v))
	for i, x := range v {
		o[i] = x
	}
	fmt.Fprintf(l.f, l.fmt, o...)
}

func (l *SensorLogger) Close() {
	l.f.Close()
}
