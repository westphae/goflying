// +build arm

package main

import (
	"encoding/json"
	"github.com/westphae/linux-mpu9150/mpu"
	"log"
	"time"
)

const SLEEP = 98

type MPU9150Listener struct {
	r    *room
	data *AHRSData
}

func (ml *MPU9150Listener) SetRoom(r *room) {
	ml.r = r
}

func (ml *MPU9150Listener) Init() {
	mpu.InitMPU(10, 0)
	ml.data = new(AHRSData)
}

func (ml *MPU9150Listener) Close() {
	mpu.CloseMPU()
}

func (ml *MPU9150Listener) GetData() *AHRSData {
	return ml.data
}

func (ml *MPU9150Listener) update() {
	log.Println("Making up some AHRS data")
	var err error
	//ml.data, err = mpu.ReadMPUAll()
	if err != nil {
		log.Println("Error reading mpu")
	}
}

func (ml *MPU9150Listener) Run() {
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
