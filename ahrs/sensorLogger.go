package ahrs

import (
	"os"
	"fmt"
	"log"
	"strings"
)

type SensorLogger struct {
	f       *os.File
	h       []string
	fmt     string
}

func NewSensorLogger(fn string, h ...string) (l SensorLogger) {
	l.h = h
	f, err := os.Create("/var/log/" + fn)
	l.f = f
	if err != nil {
		log.Fatalln(err)
	}

	fmt.Fprint(l.f, strings.Join(l.h, ","), "\n")
	s := strings.Repeat("%f,", len(l.h))
	l.fmt = strings.Join([]string{s[:len(s)-1], "\n"}, "")
	return
}

func (l *SensorLogger) Log(v ...interface{}) {
	fmt.Fprintf(l.f, l.fmt, v...)
}

func (l *SensorLogger) Close() {
	l.f.Close()
}
