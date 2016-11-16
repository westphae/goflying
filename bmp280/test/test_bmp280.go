package main

import (
	"fmt"
	"time"
	"github.com/westphae/goflying/bmp280"
)

func main() {
	var (
		bmp      *bmp280.BMP280
		cur      *bmp280.BMPData
		err      error
	)

	bmp, err = bmp280.NewBMP280(bmp280.Address1, bmp280.NormalMode, bmp280.StandbyTime63ms, bmp280.FilterCoeff16, bmp280.Oversamp16x, bmp280.Oversamp16x)
	if err != nil {
		fmt.Printf("Error: couldn't initialize BMP280: %s\n", err)
		return
	}

	fmt.Printf("DigT1: %d, DigT2: %d, DigT3: %d\n", bmp.DigT[1], bmp.DigT[2], bmp.DigT[3])
	fmt.Printf("DigP1: %d, DigP2: %d, DigP3: %d\n", bmp.DigP[1], bmp.DigP[2], bmp.DigP[3])
	fmt.Printf("DigP4: %d, DigP5: %d, DigP6: %d\n", bmp.DigP[4], bmp.DigP[5], bmp.DigP[6])
	fmt.Printf("DigP7: %d, DigP8: %d, DigP9: %d\n", bmp.DigP[7], bmp.DigP[8], bmp.DigP[9])

	defer bmp.Close()
	fmt.Println("BMP280 initialized successfully")

	clock := time.NewTicker(bmp.Delay)
	for {
		<-clock.C

		cur = <-bmp.C
		fmt.Printf("\nTime:   %v\n", cur.T)
		fmt.Printf("Temperature: %3.2f\n", cur.Temperature)
		fmt.Printf("Pressure:   %4.2f\n", cur.Pressure)
		fmt.Printf("Altitude:  %5.1f\n", cur.Altitude)
	}
}
