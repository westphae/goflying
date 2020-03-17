package main

import (
	"fmt"
	"time"

	"github.com/kidoman/embd"
	"github.com/westphae/goflying/bmp280"
)

func main() {
	var cur *bmp280.BMPData

	i2cbus := embd.NewI2CBus(1)

	var bmps []*bmp280.BMP280
	for i, address := range []byte{bmp280.Address1, bmp280.Address2} {
		bmp, err := bmp280.NewBMP280(&i2cbus, address,
			bmp280.NormalMode, bmp280.StandbyTime63ms, bmp280.FilterCoeff16, bmp280.Oversamp16x, bmp280.Oversamp16x)
		if err != nil {
			fmt.Printf("no BMP280 at address %d: %s\n", i, err)
			continue
		}
		bmps = append(bmps, bmp)
		defer bmp.Close()
	}
	if len(bmps) == 0 {
		return
	}

	fmt.Println("t,chip,temp,press,alt")
	clock := time.NewTicker(bmps[0].Delay)
	for {
		<-clock.C
		for _, bmp := range bmps {
			cur = <-bmp.C
			fmt.Printf("%v,%X,%3.2f,%4.2f,%5.1f\n", cur.T, bmp.Address, cur.Temperature, cur.Pressure, bmp280.CalcAltitude(cur.Pressure))
		}
	}
}
