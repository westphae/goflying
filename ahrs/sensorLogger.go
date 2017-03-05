package ahrs

import (
	"fmt"
	"log"
	"os"
	"strings"
)

type AHRSLogger struct {
	f      *os.File
	logMap map[string]interface{}
	Header []string
	fmt    string
	vals   []interface{}
}

func NewAHRSLogger(filename string, logMap map[string]interface{}) (l *AHRSLogger) {
	l = new(AHRSLogger)
	f, err := os.Create(filename)
	if err != nil {
		log.Fatalln(err)
	}
	l.f = f
	l.logMap = logMap

	l.Header = make([]string, len(logMap))
	i := 0
	for k := range l.logMap {
		l.Header[i] = k
		i++
	}

	fmt.Fprint(l.f, strings.Join(l.Header, ","), "\n")
	s := strings.Repeat("%f,", len(l.Header))
	l.fmt = strings.Join([]string{s[:len(s)-1], "\n"}, "")
	l.vals = make([]interface{}, len(l.Header))
	return
}

func (l *AHRSLogger) Log() {
	for i, k := range l.Header {
		l.vals[i] = (l.logMap)[k]
	}
	fmt.Fprintf(l.f, l.fmt, l.vals...)
}

func (l *AHRSLogger) Close() {
	l.f.Close()
}
