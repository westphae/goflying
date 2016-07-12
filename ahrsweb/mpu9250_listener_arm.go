// +build arm

package main

import (
	"encoding/json"
	"github.com/westphae/goflying/mpu9250"
	"log"
	"time"
	"math"
)

type MPU9250Listener struct {
	r	*room
	mpu 	*mpu9250.MPU9250
	data	*AHRSData
}

func (ml *MPU9250Listener) SetRoom(r *room) {
	ml.r = r
}

const updateFreq = 20

func (ml *MPU9250Listener) Init() {
	ml.mpu = mpu9250.NewMPU9250(250, 4, updateFreq)
	ml.data = new(AHRSData)
	time.Sleep(100 * time.Millisecond)
}

func (ml *MPU9250Listener) Close() {
	ml.mpu.CloseMPU()
}

func (ml *MPU9250Listener) GetData() *AHRSData {
	return ml.data
}

func (ml *MPU9250Listener) update() {
	ml.data.Ts, ml.data.Gx, ml.data.Gy, ml.data.Gz,
		ml.data.Ax, ml.data.Ay, ml.data.Az,
		ml.data.Mx, ml.data.My, ml.data.Mz = ml.mpu.Read()

	// Quick and dirty calcs
	ml.data.Tsm = ml.data.Ts
	if !math.IsNaN(ml.data.Gy) {
		ml.data.Pitch += 0.1 * ml.data.Gy
	}
	if !math.IsNaN(ml.data.Gx) {
		ml.data.Roll += 0.1 * ml.data.Gx
	}
	if !math.IsNaN(ml.data.Gz) {
		ml.data.Heading += 0.1 * ml.data.Gz
	}
	ml.data.X_accel = ml.data.Ax
	ml.data.Y_accel = ml.data.Ay
	ml.data.Z_accel = ml.data.Az
	ml.data.X_mag = ml.data.Mx
	ml.data.Y_mag = ml.data.My
	ml.data.Z_mag = ml.data.Mz
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
