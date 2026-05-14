// Package bmp280 adapts the kernel IIO BMP280 driver (via github.com/westphae/go-iio/bmp280)
// onto the channel-based contract goflying expects (sensors.BMPData on .C and .CBuf).
// The kernel driver handles compensation; we just poll the sysfs-exposed values and
// republish them on the legacy channels.
package bmp280

import (
	"context"
	"fmt"
	"log"
	"math"
	"os"
	"path/filepath"
	"strings"
	"time"

	goiiobmp "github.com/westphae/go-iio/bmp280"
	"github.com/westphae/goflying/sensors"
)

const (
	QNH     = 1013.25 // Sea level reference pressure in hPa
	bufSize = 256     // Depth of the buffered history channel
	pollHz  = 10      // Poll rate; matches the legacy driver's effective rate at typical settings
)

// BMP280 wraps an open IIO BMP280 device and publishes samples on two channels:
// C is unbuffered (latest reading, non-blocking publish), CBuf is a 256-deep ring.
type BMP280 struct {
	Address byte // I²C address hint kept for legacy callers (0x76 or 0x77)

	dev    *goiiobmp.BMP280
	cancel context.CancelFunc
	t0     time.Time

	C    <-chan *sensors.BMPData
	CBuf <-chan *sensors.BMPData
}

// NewBMP280 opens the kernel-managed BMP280 IIO device and starts a goroutine
// that polls it and publishes sensors.BMPData on C and CBuf.
//
// address is one of bmp280.Address1 (0x76) or bmp280.Address2 (0x77). With a
// single sensor on the bus it is honored as a hint; with two BMP280s it
// selects the matching sysfs path.
//
// oversampTemp and oversampPress accept the bmp280.OversampNx constants and
// are translated to the integer oversampling ratios accepted by IIO.
func NewBMP280(address, oversampTemp, oversampPress byte) (*BMP280, error) {
	opts := []goiiobmp.Option{
		goiiobmp.WithOversampling(decodeOversamp(oversampTemp), decodeOversamp(oversampPress)),
	}
	if path, ok := iioPathForAddress(address); ok {
		opts = append(opts, goiiobmp.WithPath(path))
	}

	dev, err := goiiobmp.Open(opts...)
	if err != nil {
		return nil, fmt.Errorf("bmp280: %w", err)
	}

	ctx, cancel := context.WithCancel(context.Background())
	cC := make(chan *sensors.BMPData)
	cBuf := make(chan *sensors.BMPData, bufSize)

	b := &BMP280{
		Address: address,
		dev:     dev,
		cancel:  cancel,
		t0:      time.Now(),
		C:       cC,
		CBuf:    cBuf,
	}
	go b.poll(ctx, cC, cBuf)
	return b, nil
}

func (b *BMP280) poll(ctx context.Context, cC, cBuf chan *sensors.BMPData) {
	defer close(cC)
	defer close(cBuf)

	ticker := time.NewTicker(time.Second / pollHz)
	defer ticker.Stop()

	for {
		select {
		case <-ctx.Done():
			return
		case t := <-ticker.C:
			s, err := b.dev.Read()
			if err != nil {
				log.Printf("bmp280: read error: %s", err)
				continue
			}
			d := &sensors.BMPData{
				Temperature: s.TempC,
				Pressure:    s.PressKPa * 10.0, // kPa -> hPa
				T:           t.Sub(b.t0),
			}
			select {
			case cC <- d:
			default:
			}
			select {
			case cBuf <- d:
			default:
			}
		}
	}
}

// Close stops the polling goroutine and releases the IIO device.
func (b *BMP280) Close() {
	b.cancel()
	_ = b.dev.Close()
}

// CalcAltitude returns altitude in feet for a pressure in hPa, using QNH=1013.25 hPa.
func CalcAltitude(press float64) float64 {
	return 145366.45 * (1.0 - math.Pow(press/QNH, 0.190284))
}

func decodeOversamp(o byte) int {
	switch o {
	case Oversamp1x:
		return 1
	case Oversamp2x:
		return 2
	case Oversamp4x:
		return 4
	case Oversamp8x:
		return 8
	case Oversamp16x:
		return 16
	default:
		return 0 // skipped or unknown — leave the kernel default in place
	}
}

// iioPathForAddress walks /sys/bus/iio/devices and returns the path of the
// first iio:deviceN whose underlying I²C bus address (1-007X) matches the
// requested address. The boolean is false when no match is found, in which
// case the caller should fall back to open-by-name.
func iioPathForAddress(address byte) (string, bool) {
	const root = "/sys/bus/iio/devices"
	entries, err := os.ReadDir(root)
	if err != nil {
		return "", false
	}
	suffix := fmt.Sprintf("-%04x", address) // e.g. "-0076"
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
