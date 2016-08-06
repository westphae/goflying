package main

import (
	"bufio"
	"encoding/csv"
	"io"
	"log"
	"os"
	"github.com/westphae/goflying/ahrs"
	"strconv"
	"errors"
	"sort"
)

type SituationFromFile struct {
	t	[]float64
	ts	[]float64
	a1	[]float64
	a2	[]float64
	a3	[]float64
	h1	[]float64
	h2	[]float64
	h3	[]float64
	m1	[]float64
	m2	[]float64
	m3	[]float64
	tw	[]float64
	w1	[]float64
	w2	[]float64
	w3	[]float64
	ta	[]float64
	alt	[]float64
}

func NewSituationFromFile (fn string) (sit *SituationFromFile, err error) {
	sit = new(SituationFromFile)
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
			switch fields[i] {
			case "T":
				sit.t = expandAppend(&sit.t, v)
			case "TS":
				sit.ts = expandAppend(&sit.ts, v)
			case "A1":
				sit.a1 = expandAppend(&sit.a1, v)
			case "A2":
				sit.a2 = expandAppend(&sit.a2, v)
			case "A3":
				sit.a3 = expandAppend(&sit.a3, v)
			case "H1":
				sit.h1 = expandAppend(&sit.h1, v)
			case "H2":
				sit.h2 = expandAppend(&sit.h2, v)
			case "H3":
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
			case "TA":
				sit.ta = expandAppend(&sit.ta, v)
			case "Alt":
				sit.alt = expandAppend(&sit.alt, v)
			default:
				continue
			}
		}
	}
	err = nil
	return
}

// BeginTime returns the time stamp when the records begin
func (s *SituationFromFile) BeginTime() (float64) {
	return s.t[0]
}

// Interpolate is only filler for reading from a sensor file: we don't know the "actual" situation since it was reality!
func (s *SituationFromFile) Interpolate(t float64, st *ahrs.State, aBias, bBias, mBias []float64) (error) {
	if t < s.t[0] || t > s.t[len(s.t)-1] {
		st = new(ahrs.State)
		return errors.New("sim: requested time is outside of recorded data")
	}
	st.E0 = 1
	st.F0 = 1
	st.T = t

	return nil
}

func (s *SituationFromFile) Measurement(t float64, m *ahrs.Measurement,
		uValid, wValid, sValid, mValid bool,
		uNoise, wNoise, aNoise, bNoise, mNoise float64,
		uBias, aBias, bBias, mBias []float64,
	) (error) {
	if t < s.t[0] || t > s.t[len(s.t)-1] {
		m = new(ahrs.Measurement)
		return errors.New("sim: requested time is outside of recorded data")
	}
	ix := 0
	if t > s.t[0] {
		ix = sort.SearchFloat64s(s.t, t) - 1
	}

	f := (s.t[ix+1] - t) / (s.t[ix+1] - s.t[ix])

	m.UValid = false
	m.U1 = 0
	m.U2 = 0
	m.U3 = 0
	m.WValid = s.tw[ix] - s.ts[ix] < 5 // Arbitrary: allow up to 5 sec lag for GPS data (5s seems typical)
	m.W1 = f*s.w1[ix] + (1-f)*s.w1[ix+1]
	m.W2 = f*s.w2[ix] + (1-f)*s.w2[ix+1]
	m.W3 = f*s.w3[ix] + (1-f)*s.w3[ix+1]
	m.A1 = -(f*s.a1[ix] + (1-f)*s.a1[ix+1])
	m.A2 = -(f*s.a2[ix] + (1-f)*s.a2[ix+1])
	m.A3 = -(f*s.a3[ix] + (1-f)*s.a3[ix+1])
	m.B1 = (f*s.h1[ix] + (1-f)*s.h1[ix+1])
	m.B2 = (f*s.h2[ix] + (1-f)*s.h2[ix+1])
	m.B3 = (f*s.h3[ix] + (1-f)*s.h3[ix+1])
	m.MValid = false // For now, just invalidate it, can add back after magnetometer data is understood
	m.M1 = f*s.m1[ix] + (1-f)*s.m1[ix+1]
	m.M2 = f*s.m2[ix] + (1-f)*s.m2[ix+1]
	m.M3 = f*s.m3[ix] + (1-f)*s.m3[ix+1]
	m.T  = t
	return nil
}
