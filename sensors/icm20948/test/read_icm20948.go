package main

import (
	"fmt"
	"time"

	"github.com/kidoman/embd"
	"github.com/westphae/goflying/sensors/icm20948"
)

func main() {
	i2cbus := embd.NewI2CBus(1)

	var icms []*icm20948.ICM20948
	for i, address := range []byte{icm20948.MPU_ADDRESS1, icm20948.MPU_ADDRESS2} {
		icm, err := icm20948.NewICM20948(&i2cbus, address, 250, 2, 1000, true, false)
		if err != nil {
			fmt.Printf("no ICM20948 at address %d: %s\n", i, err)
			continue
		}
		icms = append(icms, icm)
	}
	if len(icms) == 0 {
		return
	}

	t0 := time.Now()
	for {
		for _, icm := range icms {
			cur := <-icm.CBuf
			fmt.Printf("%.3f,%X,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.3f,%.2f,%.2f,%.2f,%.1f\n", float64(cur.T.Sub(t0))/1e9, icm.Address, cur.A1, cur.A2, cur.A3, cur.G1, cur.G2, cur.G3, float64(cur.TM.Sub(t0))/1e9, cur.M1, cur.M2, cur.M3, cur.Temp)
		}
	}
}
