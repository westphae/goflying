// +build arm

package main

import (
	"encoding/json"
	"github.com/westphae/goflying/mpu9250"
	"log"
	"time"
)

type MPU9250Listener struct {
	r	*room
	mpu 	*mpu9250.MPU9250
	data	*AHRSData
}

func (ml *MPU9250Listener) SetRoom(r *room) {
	ml.r = r
}

func (ml *MPU9250Listener) Init() {
	ml.mpu = mpu9250.NewMPU9250(100, 250, 2)
	ml.data = new(AHRSData)
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
}

func (ml *MPU9250Listener) Run() {
	clock := time.NewTicker(100 * time.Millisecond)

	for {
		<-clock.C

		ml.update()

		msg, err := json.Marshal(ml.GetData())
		if err != nil {
			log.Println("Error marshalling json data: ", err)
			continue
		}

		log.Println("Sending IMUData to room")
		ml.r.forward <- msg
	}
}
