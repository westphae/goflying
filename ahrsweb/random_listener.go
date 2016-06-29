package main

import (
	"encoding/json"
	"log"
	"math/rand"
	"time"
	//"github.com/westphae/linux-mpu9150/mpu"
)

const SLEEP = 98

type RandListener struct {
	r    *room
	data *AHRSData
}

func (ml *RandListener) SetRoom(r *room) {
	ml.r = r
}

func (ml *RandListener) Init() {
	ml.data = new(AHRSData)
}

func (ml *RandListener) Close() {
}

func (ml *RandListener) GetData() *AHRSData {
	return ml.data
}

func (ml *RandListener) update() {
	log.Println("Making up some AHRS data")
	ml.data.Pitch += 0.0 * rand.Float32()
	ml.data.Roll -= 0.4 * rand.Float32()
	ml.data.Heading += rand.Float32()
	ml.data.Gx = int16(rand.Int())
	ml.data.Gy = int16(rand.Int())
	ml.data.Gz = int16(rand.Int())
	ml.data.Ax = int16(rand.Int())
	ml.data.Ay = int16(rand.Int())
	ml.data.Az = int16(rand.Int())
	ml.data.Qw = rand.Int31()
	ml.data.Qx = rand.Int31()
	ml.data.Qy = rand.Int31()
	ml.data.Qz = rand.Int31()
	ml.data.Mx = int16(rand.Int())
	ml.data.My = int16(rand.Int())
	ml.data.Mz = int16(rand.Int())
	ml.data.Ts = rand.Uint32()
	ml.data.Tsm = rand.Uint32()
	ml.data.X_accel = int16(rand.Int())
	ml.data.Y_accel = int16(rand.Int())
	ml.data.Z_accel = int16(rand.Int())
	ml.data.X_mag = int16(rand.Int())
	ml.data.Y_mag = int16(rand.Int())
	ml.data.Z_mag = int16(rand.Int())
}

func (ml *RandListener) Run() {
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
