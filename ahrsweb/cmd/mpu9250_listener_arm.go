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
	ml.data.E0 = 1
	ml.data.F0 = 1
	ml.data.UValid = false
	ml.data.WValid = false
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
		ml.data.SValid = true
		ml.data.T = float64(data.T.UnixNano()/1000)/1e6
		ml.data.B1, ml.data.B2, ml.data.B3 = data.G1, data.G2, data.G3
		ml.data.A1, ml.data.A2, ml.data.A3 = data.A1, data.A2, data.A3

		// Quick and dirty calcs for demonstration
		ml.data.Pitch += 0.1 * ml.data.B1
		ml.data.Roll += 0.1 * ml.data.B2
		ml.data.Heading -= 0.1 * ml.data.B3
	} else {
		ml.data.SValid = false
	}

	if data.MagError == nil && data.NM > 0 {
		ml.data.MValid = true
		ml.data.M1, ml.data.M2, ml.data.M3 = data.M1, data.M2, data.M3
	} else {
		ml.data.MValid = false
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
