package main

import (
	"fmt"
	"time"

	"github.com/kidoman/embd"
	"github.com/westphae/goflying/sensors/mpu9250"
)

func main() {
	i2cbus := embd.NewI2CBus(1)

	var mpus []*mpu9250.MPU9250
	for i, address := range []byte{mpu9250.MPU_ADDRESS1, mpu9250.MPU_ADDRESS2} {
		mpu, err := mpu9250.NewMPU9250(&i2cbus, address, 250, 4, 1000, true, false)
		if err != nil {
			fmt.Printf("no MPU9250 at address %d: %s\n", i, err)
			continue
		}
		mpus = append(mpus, mpu)
	}
	if len(mpus) == 0 {
		return
	}

	t0 := time.Now()
	for {
		for _, mpu := range mpus {
			cur := <-mpu.CBuf
			fmt.Printf("%.3f,%X,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.3f,%.2f,%.2f,%.2f,%.1f\n", float64(cur.T.Sub(t0))/1e9, mpu.Address, cur.A1, cur.A2, cur.A3, cur.G1, cur.G2, cur.G3, float64(cur.TM.Sub(t0))/1e9, cur.M1, cur.M2, cur.M3, cur.Temp)
		}
	}
}
