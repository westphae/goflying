package main

import (
	"encoding/json"
	"log"
	"math/rand"
	"time"
	//"github.com/westphae/linux-mpu9150/mpu"
)

type SimListener struct {
	r    *room
	data *AHRSData
}

func (ml *SimListener) SetRoom(r *room) {
	ml.r = r
}

func (ml *SimListener) Init() {
	ml.data = new(AHRSData)
}

func (ml *SimListener) Close() {
}

func (ml *SimListener) GetData() *AHRSData {
	return ml.data
}

func (ml *SimListener) update() {
	log.Println("Making up some AHRS data")
	ml.data.Pitch += 0.0 * rand.Float64()
	ml.data.Roll -= 0.4 * rand.Float64()
	ml.data.Heading += rand.Float64()
	ml.data.Gx = rand.Float64()
	ml.data.Gy = rand.Float64()
	ml.data.Gz = rand.Float64()
	ml.data.Ax = rand.Float64()
	ml.data.Ay = rand.Float64()
	ml.data.Az = rand.Float64()
	ml.data.Qw = rand.Float64()
	ml.data.Qx = rand.Float64()
	ml.data.Qy = rand.Float64()
	ml.data.Qz = rand.Float64()
	ml.data.Mx = rand.Float64()
	ml.data.My = rand.Float64()
	ml.data.Mz = rand.Float64()
	ml.data.Ts = rand.Int63()
	ml.data.Tsm = rand.Int63()
	ml.data.X_accel = rand.Float64()
	ml.data.Y_accel = rand.Float64()
	ml.data.Z_accel = rand.Float64()
	ml.data.X_mag = rand.Float64()
	ml.data.Y_mag = rand.Float64()
	ml.data.Z_mag = rand.Float64()
}

func (ml *SimListener) Run() {
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
