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
		mpu, err = mpu9250.NewMPU9250(250, 4, 100, true, false)
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

	var data *mpu9250.MPUData

	for {
		<-clock.C

		data = <-mpu.CAvg
		fmt.Printf("\nTime:   %6.1f ms\n", float64(data.DT.Nanoseconds())/1000000)
		fmt.Printf("Number of Observations: %d\n", data.N)
		fmt.Printf("Gyro:   % +8.1f % +8.1f % +8.1f\n", data.G1, data.G2, data.G3)
		fmt.Printf("Accel:  % +8.2f % +8.2f % +8.2f\n", data.A1, data.A2, data.A3)

		if !mpu.MagEnabled() {
			fmt.Println("Magnetometer disabled")
		} else if data.MagError != nil {
			fmt.Println(data.MagError.Error())
		} else {
			fmt.Printf("Mag:    % +8.0f % +8.0f % +8.0f\n", data.M1, data.M2, data.M3)
		}
	}
}
