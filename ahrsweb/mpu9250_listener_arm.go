// +build arm

package main

import (
	"encoding/json"
	"github.com/westphae/goflying/mpu9250"
	"log"
	"time"
	"fmt"
	"os"
)

type MPU9250Listener struct {
	r	*room
	mpu 	*mpu9250.MPU9250
	data	*AHRSData
}

func (ml *MPU9250Listener) SetRoom(r *room) {
	ml.r = r
}

const (
	gyroRange = 250
	accelRange = 4
	updateFreq = 100
	MPURETRIES = 10
)

func (ml *MPU9250Listener) Init() {
	var err error

	for i:=0; i<MPURETRIES; i++ {
		mpu, err := mpu9250.NewMPU9250(gyroRange, accelRange, updateFreq, true, false)
		if err != nil {
			fmt.Printf("Error initializing MPU9250, attempt %d of %d\n", i, MPURETRIES)
			time.Sleep(5 * time.Second)
		} else {
			mpu.CCal <- 2
			err = <-mpu.CCalResult
			if err != nil {
				fmt.Printf("Error calibrating try %d of %d\n", i, MPURETRIES)
				time.Sleep(100 * time.Millisecond)
			} else {
				ml.mpu = mpu
				break
			}
		}
	}

	if err != nil {
		fmt.Println("Error: couldn't initialize MPU9250")
		os.Exit(1)
	}

	for i:=0; i<MPURETRIES; i++ {
	}

	if err != nil {
		fmt.Println("Error: couldn't calibrate MPU9250")
		os.Exit(2)
	}

	ml.data = new(AHRSData)
	time.Sleep(250 * time.Millisecond)
}

func (ml *MPU9250Listener) Close() {
	ml.mpu.CloseMPU()
}

func (ml *MPU9250Listener) GetData() *AHRSData {
	return ml.data
}

func (ml *MPU9250Listener) update() {
	data := <-ml.mpu.CAvg

	if data.GAError == nil && data.N > 0 {
		ml.data.Ts = data.T.UnixNano()
		ml.data.Gx, ml.data.Gy, ml.data.Gz = data.G1, data.G2, data.G3
		ml.data.Ax, ml.data.Ay, ml.data.Az = data.A1, data.A2, data.A3

		// Quick and dirty calcs
		ml.data.Pitch += 0.1 * ml.data.Gx
		ml.data.Roll += 0.1 * ml.data.Gy
		ml.data.Heading -= 0.1 * ml.data.Gz

		ml.data.X_accel = ml.data.Ax
		ml.data.Y_accel = ml.data.Ay
		ml.data.Z_accel = ml.data.Az
	}

	if data.MagError == nil && data.NM > 0 {
		ml.data.Tsm = data.TM.UnixNano()
		ml.data.Mx, ml.data.My, ml.data.Mz = data.M1, data.M2, data.M3

		ml.data.X_mag = ml.data.Mx
		ml.data.Y_mag = ml.data.My
		ml.data.Z_mag = ml.data.Mz
	}
	if ml.data.Az < 0.2 {
		os.Exit(0)
	}
}

func (ml *MPU9250Listener) Run() {
	clock := time.NewTicker(100 * time.Millisecond)

	for {
		<-clock.C

		ml.update()

		data := ml.GetData()
		msg, err := json.Marshal(data)
		if err != nil {
			log.Println("Error marshalling json data: ", err.Error())
			log.Println(data)
			continue
		}

		log.Println("Sending IMUData to room")
		ml.r.forward <- msg
	}
}
