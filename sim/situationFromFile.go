package main

import (
	"bufio"
	"encoding/csv"
	"errors"
	"io"
	"log"
	"os"
	"strconv"

	"../ahrs"
	"github.com/skelterjohn/go.matrix"
)

type SituationFromFile struct {
	ix     int
	t      []float64
	a1     []float64
	a2     []float64
	a3     []float64
	h1     []float64
	h2     []float64
	h3     []float64
	m1     []float64
	m2     []float64
	m3     []float64
	tw     []float64
	w1     []float64
	w2     []float64
	w3     []float64
	wvalid []float64
	alt    []float64
	logMap map[string][]*float64 // Map only for analysis/debugging
	logMapCurrent map[string]interface{}
}

func NewSituationFromFile(fn string) (sit *SituationFromFile, err error) {
	sit = new(SituationFromFile)
	sit.logMap = make(map[string][]*float64)
	sit.logMapCurrent = make(map[string]interface{})
	f, err := os.Open(fn)
	if err != nil {
		log.Fatalln(err.Error())
	}
	defer f.Close()
	r := csv.NewReader(bufio.NewReader(f))

	// Read header line
	rec, err := r.Read()
	if err != nil {
		log.Fatalln(err.Error())
	}

	var fields map[int]string = make(map[int]string)
	for i, k := range rec {
		fields[i] = k
	}

	var expandAppend = func(a *[]float64, v float64) (x []float64) {
		n := len(*a)
		if n == cap(*a) {
			x := append(*a, make([]float64, 0, 16384)...)
			//x := make([]float64, 0, cap(*a)+16384)
			//copy(x, *a)
			a = &x
		}
		x = append(*a, v)
		//x = (*a)[:n+1]
		//x[n] = v
		return
	}

	// Read the rest of the data into the situation
	var (
		j   int
		t0  float64
	)
	for {
		rec, err = r.Read()
		if err == io.EOF {
			break
		} else if err != nil {
			log.Printf("csv %s, skipping this one\n", err.Error())
			continue
		}

		for i, k := range rec {
			v, err := strconv.ParseFloat(k, 64)
			if err != nil {
				log.Printf("csv contains bad data %s, skipping this one\n", err.Error())
			}
			sit.logMap[fields[i]] = append(sit.logMap[fields[i]], &v)
			switch fields[i] {
			case "T":
				if j == 0 {
					t0 = v
				}
				sit.t = expandAppend(&sit.t, v)
			case "A1":
				sit.a1 = expandAppend(&sit.a1, v)
			case "A2":
				sit.a2 = expandAppend(&sit.a2, v)
			case "A3":
				sit.a3 = expandAppend(&sit.a3, v)
			case "B1":
				sit.h1 = expandAppend(&sit.h1, v)
			case "B2":
				sit.h2 = expandAppend(&sit.h2, v)
			case "B3":
				sit.h3 = expandAppend(&sit.h3, v)
			case "M1":
				sit.m1 = expandAppend(&sit.m1, v)
			case "M2":
				sit.m2 = expandAppend(&sit.m2, v)
			case "M3":
				sit.m3 = expandAppend(&sit.m3, v)
			case "TW":
				sit.tw = expandAppend(&sit.tw, v)
			case "W1":
				sit.w1 = expandAppend(&sit.w1, v)
			case "W2":
				sit.w2 = expandAppend(&sit.w2, v)
			case "W3":
				sit.w3 = expandAppend(&sit.w3, v)
			case "WValid":
				sit.wvalid = expandAppend(&sit.wvalid, v)
			case "Alt":
				sit.alt = expandAppend(&sit.alt, v)
			default:
				continue
			}
		}
		sit.t[j] -= t0
		sit.tw[j] -= t0
		j += 1
	}
	err = nil
	log.Printf("Records read: %d\n", j)
	return
}

// BeginTime returns the time stamp when the records begin.
func (s *SituationFromFile) BeginTime() float64 {
	for k, v := range s.logMap {
		s.logMapCurrent[k] = *v[0]
	}
	return s.t[0]
}

// NextTime returns the next time stamp available.
func (s *SituationFromFile) NextTime() (err error) {
	if s.ix >= len(s.t)-1 {
		return errors.New("sim: requested time is outside of recorded data")
	}
	s.ix += 1
	for s.ix+1 < len(s.t) && s.t[s.ix] <= s.t[s.ix-1] { // No duplicates
		s.ix += 1
	}
	return nil
}

// UpdateState is only filler for reading from a sensor file: we don't know the "actual" situation since it was reality!
func (s *SituationFromFile) UpdateState(st *ahrs.State, aBias, bBias, mBias []float64) error {
	st.E0 = 1
	st.F0 = 1
	for k, v := range s.logMap {
		s.logMapCurrent[k] = *v[s.ix]
	}

	return nil
}

func (s *SituationFromFile) UpdateMeasurement(m *ahrs.Measurement,
		uValid, wValid, sValid, mValid bool,
		uNoise, wNoise, aNoise, bNoise, mNoise float64,
		uBias, aBias, bBias, mBias []float64) error {
	m.U1 = 0
	m.U2 = 0
	m.U3 = 0
	m.UValid = false
	m.TU = 0
	m.W1 = s.w1[s.ix]
	m.W2 = s.w2[s.ix]
	m.W3 = s.w3[s.ix]
	if s.wvalid[s.ix] < 0.5 {
		m.WValid = false
	} else {
		m.WValid = true
	}
	m.TW = s.tw[s.ix]
	m.A1 = s.a1[s.ix]
	m.A2 = s.a2[s.ix]
	m.A3 = s.a3[s.ix]
	m.B1 = s.h1[s.ix]
	m.B2 = s.h2[s.ix]
	m.B3 = s.h3[s.ix]
	m.SValid = true
	m.M1 = s.m1[s.ix]
	m.M2 = s.m2[s.ix]
	m.M3 = s.m3[s.ix]
	m.MValid = m.M1 != 0 || m.M2 != 0 || m.M3 != 0
	m.T = s.t[s.ix]

	m.M = matrix.Zeros(15, 15)
	return nil
}

func (s *SituationFromFile) GetLogMap() (p map[string]interface{}) {
	for k, v := range s.logMap {
		s.logMapCurrent[k] = *v[0]
	}
	return s.logMapCurrent
}
