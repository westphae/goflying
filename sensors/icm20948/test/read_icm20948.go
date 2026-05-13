package main

import (
	"bufio"
	"fmt"
	"io"
	"log"
	"math"
	"os"
	"os/signal"
	"strings"
	"sync"
	"syscall"
	"time"

	"github.com/westphae/goflying/sensors"
	"github.com/westphae/goflying/sensors/icm20948"
)

const (
	ansiHome        = "\033[H"
	ansiClearScreen = "\033[2J"
	ansiClearLine   = "\033[K"
	ansiHideCursor  = "\033[?25l"
	ansiShowCursor  = "\033[?25h"

	ewmaTau        = 1.0 * time.Second // EWMA time constant (~1 Hz bandwidth)
	redrawInterval = 100 * time.Millisecond
	maxErrors      = 5
)

type errorLog struct {
	mu        sync.Mutex
	startTime time.Time
	total     int
	recent    []string  // last maxErrors messages for display
	lastTime  time.Time // time of most recent error (zero if none yet)
}

type errorStats struct {
	total     int           // cumulative event count since start
	perMinute float64       // cumulative session average: total × 60 / elapsed
	lastAgo   time.Duration // since most recent event; zero if never
	recent    []string
}

func (e *errorLog) add(line string) {
	e.mu.Lock()
	defer e.mu.Unlock()
	e.total++
	e.lastTime = time.Now()
	e.recent = append(e.recent, line)
	if len(e.recent) > maxErrors {
		e.recent = e.recent[len(e.recent)-maxErrors:]
	}
}

func (e *errorLog) snapshot() errorStats {
	e.mu.Lock()
	defer e.mu.Unlock()
	now := time.Now()
	out := errorStats{
		total:  e.total,
		recent: append([]string(nil), e.recent...),
	}
	if elapsed := now.Sub(e.startTime).Seconds(); elapsed > 0 {
		out.perMinute = float64(e.total) * 60.0 / elapsed
	}
	if !e.lastTime.IsZero() {
		out.lastAgo = now.Sub(e.lastTime)
	}
	return out
}

func (e *errorLog) capture(r io.Reader) {
	scanner := bufio.NewScanner(r)
	for scanner.Scan() {
		e.add(scanner.Text())
	}
}

type chipState struct {
	cur             sensors.IMUData
	ewma            [10]float64 // A1..A3, G1..G3, M1..M3, Temp
	ewmaInitialized bool
	lastT           time.Time
}

func (cs *chipState) update(cur *sensors.IMUData) {
	cs.cur = *cur
	samples := [10]float64{
		cur.A1, cur.A2, cur.A3,
		cur.G1, cur.G2, cur.G3,
		cur.M1, cur.M2, cur.M3,
		cur.Temp,
	}
	now := time.Now()
	if !cs.ewmaInitialized {
		cs.ewma = samples
		cs.ewmaInitialized = true
	} else {
		dt := now.Sub(cs.lastT).Seconds()
		alpha := 1.0 - math.Exp(-dt/ewmaTau.Seconds())
		for i := range cs.ewma {
			cs.ewma[i] += alpha * (samples[i] - cs.ewma[i])
		}
	}
	cs.lastT = now
}

func main() {
	var icms []*icm20948.ICM20948
	for i, address := range []byte{icm20948.MPU_ADDRESS1, icm20948.MPU_ADDRESS2} {
		icm, err := icm20948.NewICM20948(address, 250, 2, 1000)
		if err != nil {
			fmt.Printf("no ICM20948 at address %d: %s\n", i, err)
			continue
		}
		icms = append(icms, icm)
	}
	if len(icms) == 0 {
		return
	}

	// Pipe driver log output (warnings, mag overflows, etc.) into the error
	// pane instead of letting it scribble over the live display.
	errs := &errorLog{startTime: time.Now()}
	pipeR, pipeW := io.Pipe()
	log.SetOutput(pipeW)
	log.SetFlags(log.Ltime) // HH:MM:SS only — date is implied
	go errs.capture(pipeR)

	// Restore terminal state on Ctrl-C / SIGTERM. We also tear down each
	// driver so the kernel buffer is disabled, the hrtimer trigger is
	// detached, and its configfs entry is removed — skipping this leaks a
	// trigger reference into the kernel module, and the next run's
	// buffer/enable=1 will dereference freed memory and panic the kernel.
	sig := make(chan os.Signal, 1)
	signal.Notify(sig, os.Interrupt, syscall.SIGTERM)
	go func() {
		<-sig
		for _, icm := range icms {
			icm.CloseMPU()
		}
		fmt.Printf("\033[%d;1H%s\n", 4+2*len(icms)+2+maxErrors, ansiShowCursor)
		os.Exit(0)
	}()

	fmt.Print(ansiClearScreen + ansiHome + ansiHideCursor)

	states := make([]chipState, len(icms))
	t0 := time.Now()
	var lastDraw time.Time

	for {
		for i, icm := range icms {
			cur := <-icm.CBuf
			states[i].update(cur)
			if time.Since(lastDraw) >= redrawInterval {
				lastDraw = time.Now()
				render(t0, icms, states, errs.snapshot())
			}
		}
	}
}

func render(t0 time.Time, icms []*icm20948.ICM20948, states []chipState, errs errorStats) {
	var b strings.Builder
	b.WriteString(ansiHome)

	hdr := fmt.Sprintf("%7s %3s  %7s %7s %7s  %8s %8s %8s  %7s  %7s %7s %7s %7s  %5s",
		"t", "adr", "A1", "A2", "A3", "G1", "G2", "G3", "tm", "M1", "M2", "M3", "|M|", "T")
	fmt.Fprintln(&b, hdr+ansiClearLine)

	for i := range icms {
		s := &states[i]
		mag := math.Sqrt(s.cur.M1*s.cur.M1 + s.cur.M2*s.cur.M2 + s.cur.M3*s.cur.M3)
		ewmaMag := math.Sqrt(s.ewma[6]*s.ewma[6] + s.ewma[7]*s.ewma[7] + s.ewma[8]*s.ewma[8])
		fmt.Fprintf(&b, "%7.3f %3X  %7.4f %7.4f %7.4f  %8.3f %8.3f %8.3f  %7.3f  %7.2f %7.2f %7.2f %7.2f  %5.1f%s\n",
			float64(s.cur.T.Sub(t0))/1e9, icms[i].Address,
			s.cur.A1, s.cur.A2, s.cur.A3,
			s.cur.G1, s.cur.G2, s.cur.G3,
			float64(s.cur.TM.Sub(t0))/1e9,
			s.cur.M1, s.cur.M2, s.cur.M3, mag,
			s.cur.Temp, ansiClearLine)
		fmt.Fprintf(&b, "%7s %3s  %7.4f %7.4f %7.4f  %8.3f %8.3f %8.3f  %7s  %7.2f %7.2f %7.2f %7.2f  %5.1f%s\n",
			"ewma", "",
			s.ewma[0], s.ewma[1], s.ewma[2],
			s.ewma[3], s.ewma[4], s.ewma[5],
			"",
			s.ewma[6], s.ewma[7], s.ewma[8], ewmaMag,
			s.ewma[9], ansiClearLine)
	}

	fmt.Fprintln(&b, ansiClearLine)
	var errHdr string
	if errs.total == 0 {
		errHdr = "Recent errors: none"
	} else {
		errHdr = fmt.Sprintf("Recent errors (%d total, %.1f/min, last %s ago):",
			errs.total, errs.perMinute, formatAgo(errs.lastAgo))
	}
	fmt.Fprintln(&b, errHdr+ansiClearLine)
	for i := 0; i < maxErrors; i++ {
		if i < len(errs.recent) {
			fmt.Fprintf(&b, "  %s%s\n", errs.recent[i], ansiClearLine)
		} else {
			fmt.Fprintln(&b, ansiClearLine)
		}
	}

	os.Stdout.WriteString(b.String())
}

func formatAgo(d time.Duration) string {
	if d < time.Minute {
		return fmt.Sprintf("%ds", int(d.Seconds()))
	}
	if d < time.Hour {
		return fmt.Sprintf("%dm%02ds", int(d.Minutes()), int(d.Seconds())%60)
	}
	return fmt.Sprintf("%dh%02dm", int(d.Hours()), int(d.Minutes())%60)
}
