// Package icm20948 is a channel-API adapter over the kernel's ICM-20948 IIO
// driver (github.com/westphae/icm20948-mod), publishing *sensors.IMUData on
// the C / CAvg / CBuf channels that the rest of goflying expects.
package icm20948

import (
	"context"
	"fmt"
	"os"
	"path/filepath"
	"strings"
	"sync"
	"time"

	goiioicm "github.com/westphae/go-iio/icm20948"
	"github.com/westphae/goflying/sensors"
)

const bufSize = 256

// ICM20948 wraps the kernel-IIO ICM-20948 driver and republishes its samples
// on the goflying channel API. The kernel owns the I²C bus, the chip init,
// the AK09916 master-mode magnetometer plumbing, and the Y/Z mag sign
// inversion — this struct is just glue.
type ICM20948 struct {
	sensors.IMUSensor
	sensors.IMUCalData

	Address byte

	dev        *goiioicm.ICM20948
	cancel     context.CancelFunc
	sampleRate int

	mu   sync.Mutex // protects avg/avgT0 in the relay goroutine's CAvg drain
	avg  sensors.IMUData
	avgN int
}

// NewICM20948 opens the kernel-managed ICM-20948 IIO device, starts a
// streaming capture at sampleRate Hz, and publishes decoded samples as
// *sensors.IMUData on C (latest, drop-on-full), CAvg (running average since
// last drain), and CBuf (256-deep ring).
//
// address selects between two chips by I²C address (0x68 or 0x69) when more
// than one ICM-20948 is bound; with a single chip it is informational.
// sensitivityGyro (250/500/1000/2000 dps) and sensitivityAccel (2/4/8/16 G)
// configure the kernel driver's full-scale range. sampleRate is the hrtimer
// trigger frequency in Hz; the AK09916 magnetometer inside the chip is
// hardcoded to 100 Hz in the kernel driver and is replicated into every
// record at higher trigger rates.
//
// Requires the iio-trig-hrtimer kernel module and CAP_SYS_ADMIN (root)
// because go-iio creates the trigger via configfs.
func NewICM20948(address byte, sensitivityGyro, sensitivityAccel, sampleRate int) (*ICM20948, error) {
	icm := &ICM20948{Address: address, sampleRate: sampleRate}
	if err := icm.IMUCalData.Load(); err != nil {
		icm.IMUCalData.Reset()
	}

	path, ok := iioPathForAddress(address)
	if !ok {
		return nil, fmt.Errorf("icm20948: no IIO device bound at I²C address 0x%02x", address)
	}
	opts := []goiioicm.Option{
		goiioicm.WithPath(path),
		goiioicm.WithAccelScale(sensitivityAccel),
		goiioicm.WithGyroScale(sensitivityGyro),
	}

	dev, err := goiioicm.Open(opts...)
	if err != nil {
		return nil, fmt.Errorf("icm20948: %w", err)
	}

	ctx, cancel := context.WithCancel(context.Background())
	samples, err := dev.Stream(ctx, goiioicm.StreamOptions{FrequencyHz: sampleRate})
	if err != nil {
		_ = dev.Close()
		cancel()
		return nil, fmt.Errorf("icm20948: %w", err)
	}

	icm.dev = dev
	icm.cancel = cancel

	cC := make(chan *sensors.IMUData, 1)
	cAvg := make(chan *sensors.IMUData, 1)
	cBuf := make(chan *sensors.IMUData, bufSize)
	icm.C, icm.CAvg, icm.CBuf = cC, cAvg, cBuf
	go icm.relay(samples, cC, cAvg, cBuf)
	return icm, nil
}

func (i *ICM20948) relay(in <-chan goiioicm.Sample, cC, cAvg, cBuf chan *sensors.IMUData) {
	defer close(cC)
	defer close(cAvg)
	defer close(cBuf)

	var (
		prev   time.Time
		prevOK bool
		avgT0  time.Time
	)
	for s := range in {
		d := &sensors.IMUData{
			G1:   s.GyroX - i.G01,
			G2:   s.GyroY - i.G02,
			G3:   s.GyroZ - i.G03,
			A1:   s.AccelX - i.A01,
			A2:   s.AccelY - i.A02,
			A3:   s.AccelZ - i.A03,
			Temp: s.TempC,
			N:    1, NM: 1,
			T:  s.Time,
			TM: s.Time,
		}
		mm1 := s.MagX - i.M01
		mm2 := s.MagY - i.M02
		mm3 := s.MagZ - i.M03
		d.M1 = i.Ms11*mm1 + i.Ms12*mm2 + i.Ms13*mm3
		d.M2 = i.Ms21*mm1 + i.Ms22*mm2 + i.Ms23*mm3
		d.M3 = i.Ms31*mm1 + i.Ms32*mm2 + i.Ms33*mm3
		if prevOK {
			d.DT = s.Time.Sub(prev)
			d.DTM = d.DT
		}
		prev = s.Time
		prevOK = true

		select {
		case cC <- d:
		default:
		}
		select {
		case cBuf <- d:
		default:
		}

		i.mu.Lock()
		if i.avgN == 0 {
			avgT0 = s.Time
		}
		i.avg.G1 += d.G1
		i.avg.G2 += d.G2
		i.avg.G3 += d.G3
		i.avg.A1 += d.A1
		i.avg.A2 += d.A2
		i.avg.A3 += d.A3
		i.avg.M1 += d.M1
		i.avg.M2 += d.M2
		i.avg.M3 += d.M3
		i.avg.Temp += d.Temp
		i.avgN++
		n := float64(i.avgN)
		a := &sensors.IMUData{
			G1:   i.avg.G1 / n,
			G2:   i.avg.G2 / n,
			G3:   i.avg.G3 / n,
			A1:   i.avg.A1 / n,
			A2:   i.avg.A2 / n,
			A3:   i.avg.A3 / n,
			M1:   i.avg.M1 / n,
			M2:   i.avg.M2 / n,
			M3:   i.avg.M3 / n,
			Temp: i.avg.Temp / n,
			N:    i.avgN, NM: i.avgN,
			T: s.Time, TM: s.Time,
			DT: s.Time.Sub(avgT0), DTM: s.Time.Sub(avgT0),
		}
		select {
		case cAvg <- a:
			i.avg = sensors.IMUData{}
			i.avgN = 0
		default:
		}
		i.mu.Unlock()
	}
}

// CloseMPU stops the streaming goroutine and releases the underlying IIO
// device. No restart path — to resume, construct a new ICM20948.
func (i *ICM20948) CloseMPU() {
	if i.cancel != nil {
		i.cancel()
	}
	if i.dev != nil {
		_ = i.dev.Close()
	}
}

// SampleRate returns the configured hrtimer trigger frequency in Hz.
func (i *ICM20948) SampleRate() int { return i.sampleRate }

// MagEnabled reports whether the magnetometer is being streamed. Always true
// with the kernel driver — every sample frame contains a magn reading.
func (i *ICM20948) MagEnabled() bool { return true }

// iioPathForAddress walks /sys/bus/iio/devices and returns the path of the
// iio:deviceN whose parent I²C bus address matches `address` (0x68 or 0x69).
// Returns ok=false when there's no exact match; NewICM20948 then refuses
// rather than silently opening some other chip — opening "the icm20948 by
// name" when caller asked for a specific address can reopen a device already
// streaming under a different address handle and stomp on its buffer state.
func iioPathForAddress(address byte) (string, bool) {
	const root = "/sys/bus/iio/devices"
	entries, err := os.ReadDir(root)
	if err != nil {
		return "", false
	}
	suffix := fmt.Sprintf("-%04x", address)
	for _, e := range entries {
		if !strings.HasPrefix(e.Name(), "iio:device") {
			continue
		}
		full := filepath.Join(root, e.Name())
		resolved, err := filepath.EvalSymlinks(full)
		if err != nil {
			continue
		}
		if strings.HasSuffix(filepath.Base(filepath.Dir(resolved)), suffix) {
			return full, true
		}
	}
	return "", false
}
