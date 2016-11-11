package main

import (
	"fmt"
	"time"
	"github.com/westphae/goflying/bmp280"
)

const (
	RETRIES = 10
	FREQ = 3
)

func main() {
	freq := 500
	if FREQ==1 {
		freq = 62500
	} else if FREQ > 1 {
		freq = (uint(4000) >> uint(7-FREQ))*1000
	}
	clock := time.NewTicker(freq * time.Microsecond)
	var (
		bmp      *bmp280.BMP280
		cur      *bmp280.BMPData
		err      error
	)

	for i:=0; i<RETRIES; i++ {
		bmp, err = bmp280.NewBMP280(3, FREQ, 4, 5, 5)
		if err != nil {
			fmt.Printf("Error initializing BMP280, attempt %d of 10\n", i)
			time.Sleep(5 * time.Second)
		} else {
			break
		}
	}
	if err != nil {
		fmt.Println("Error: couldn't initialize BMP280")
		return
	}
	bmp.Close()
	fmt.Println("BMP280 initialized successfully")

	for {
		<-clock.C

		cur = <-bmp.C
		fmt.Printf("\nTime:   %v\n", cur.T)
		fmt.Printf("Temperature: %3.1f\n", cur.Temperature)
		fmt.Printf("Pressure: %4.1f\n", cur.Pressure)
		fmt.Printf("Altitude: %5.0f\n", cur.Altitude)
	}
}
