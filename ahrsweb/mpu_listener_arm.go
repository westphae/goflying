// +build arm

package main

import (
	"encoding/json"
	"github.com/westphae/linux-mpu9150/mpu"
	"log"
	"time"
)

const SLEEP = 98

type MPUListener struct {
	r    *room
	data *AHRSData
}

func (ml *MPUListener) SetRoom(r *room) {
	ml.r = r
}

func (ml *MPUListener) Init() {
	mpu.InitMPU(10, 0)
	ml.data = new(AHRSData)
}

func (ml *MPUListener) Close() {
	mpu.CloseMPU()
}

func (ml *MPUListener) GetData() *AHRSData {
	return ml.data
}

func (ml *MPUListener) update() {
	log.Println("Making up some AHRS data")
	var err error
	ml.mpu, err = mpu.ReadMPUAll()
	if err != nil {
		log.Println("Error reading mpu")
	}
	ml.data.Z_mag = int16(rand.Int())
}

func (ml *MPUListener) Run() {
	for {
		ml.update()

		msg, err := json.Marshal(ml.GetData())
		if err != nil {
			log.Println("Error marshalling json data: ", err)
			continue
		}

		log.Println("Sending IMUData to room")
		ml.r.forward <- msg

		time.Sleep(time.Duration(100) * time.Millisecond)
	}
}
