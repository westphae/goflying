package main

import (
	"fmt"
	"math"
	"time"
	"github.com/westphae/goflying/mpu9250"
)

const DEG = 180/math.Pi

func main() {
	clock := time.NewTicker(100 * time.Millisecond)
	var mpu = mpu9250.NewMPU9250(500, 4, 100)
	var t0 = time.Now().UnixNano()

	for {
		<-clock.C

		t, g1, g2, g3, a1, a2, a3, m1, m2, m3 := mpu.Read()
		fmt.Printf("\nTime:   %6.1f\n", float64(t-t0)/1e6)
		fmt.Printf("Gyro:   % +8.1f % +8.1f % +8.1f\n", g1, g2, g3)
		fmt.Printf("Accel:  % +8.2f % +8.2f % +8.2f\n", a1, a2, a3)
		fmt.Printf("Mag:    % +8.0f % +8.0f % +8.0f\n", m1, m2, m3)
		t0 = t
	}
}
