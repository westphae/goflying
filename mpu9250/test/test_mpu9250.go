package main

import (
	"fmt"
	"time"
	"github.com/westphae/goflying/mpu9250"
)

func main() {
	clock := time.NewTicker(100 * time.Millisecond)
	var mpu *mpu9250.MPU9250
	var err error

	for i:=0; i<10; i++ {
		mpu, err = mpu9250.NewMPU9250(1000, 8, 100, false)
		if err != nil {
			fmt.Printf("Error initializing MPU9250, attempt %d of n\n", i)
			time.Sleep(5 * time.Second)
		} else {
			break
		}
	}

	if err != nil {
		fmt.Println("Error: couldn't initialize MPU9250")
		return
	}

	if err := mpu.CalibrateGyro(1); err != nil {
		fmt.Println(err.Error())
		return
	}

	if err := mpu.CalibrateAccel(1); err != nil {
		fmt.Println(err.Error())
		return
	}


	if err := mpu.Calibrate(1); err != nil {
		fmt.Println(err.Error())
		return
	}

	var t0 = time.Now().UnixNano()

	for {
		<-clock.C

		t, g1, g2, g3, a1, a2, a3, m1, m2, m3, gaError, magError := mpu.Read()
		fmt.Printf("\nTime:   %6.1f\n", float64(t-t0)/1e6)
		if gaError != nil {
			fmt.Printf("Error %s reading gyro/accel\n\n", gaError.Error())
		} else {
			fmt.Printf("Gyro:   % +8.1f % +8.1f % +8.1f\n", g1, g2, g3)
			fmt.Printf("Accel:  % +8.2f % +8.2f % +8.2f\n", a1, a2, a3)
		}

		if magError != nil {
			fmt.Printf("Error %s reading magnetometer\n", magError.Error())
		} else {
			fmt.Printf("Mag:    % +8.0f % +8.0f % +8.0f\n", m1, m2, m3)
		}

		t0 = t
	}
}
