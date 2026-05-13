package main

import (
	"fmt"
	"time"

	"github.com/westphae/goflying/sensors"
	"github.com/westphae/goflying/sensors/bmp280"
)

func main() {
	var cur, last *sensors.BMPData

	var bmps []*bmp280.BMP280
	for i, address := range []byte{bmp280.Address1, bmp280.Address2} {
		bmp, err := bmp280.NewBMP280(address, bmp280.Oversamp1x, bmp280.Oversamp16x)
		if err != nil {
			fmt.Printf("no BMP280 at address %d (0x%X): %s\n", i, address, err)
			continue
		}
		bmps = append(bmps, bmp)
		defer bmp.Close()
	}
	if len(bmps) == 0 {
		return
	}
	last = &sensors.BMPData{}

	fmt.Println("t,chip,dt,temp,press,alt")
	clock := time.NewTicker(100 * time.Millisecond)
	for {
		<-clock.C
		for _, bmp := range bmps {
			cur = <-bmp.C
			fmt.Printf("%v,%X,%v,%.2f,%.2f,%.1f\n",
				cur.T, bmp.Address, cur.T-last.T,
				cur.Temperature, cur.Pressure, bmp280.CalcAltitude(cur.Pressure))
			last = cur
		}
	}
}
