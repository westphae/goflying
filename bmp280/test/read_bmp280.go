package main

import (
	"../../bmp280"
	"fmt"
	"github.com/kidoman/embd"
	"time"
)

func main() {
	var cur *bmp280.BMPData

	i2cbus := embd.NewI2CBus(1)
	bmp, err := bmp280.NewBMP280(&i2cbus, bmp280.Address1,
		bmp280.NormalMode, bmp280.StandbyTime63ms, bmp280.FilterCoeff16, bmp280.Oversamp16x, bmp280.Oversamp16x)
	if err != nil {
		fmt.Printf("Error: couldn't initialize BMP280: %s\n", err)
		return
	}

	defer bmp.Close()
	//fmt.Println("BMP280 initialized successfully")
	fmt.Println("t,temp,press,alt")

	clock := time.NewTicker(bmp.Delay)
	for {
		<-clock.C
		cur = <-bmp.C
		fmt.Printf("%v,%3.2f,%4.2f,%5.1f\n", cur.T, cur.Temperature, cur.Pressure, bmp280.CalcAltitude(cur.Pressure))
	}
}
