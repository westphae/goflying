package main

import (
	"log"
	"math/rand"
	"time"
	"encoding/json"
	//"github.com/westphae/linux-mpu9150/mpu"
)

// Loop:
// Sleep 98 ms
// Poll mpu and populate mpu data struct
// Send to room.forward

const (
	SLEEP = 98
)

type IMUData struct {
	Pitch, Roll, Heading float32
	Gx, Gy, Gz, Ax, Ay, Az int16
	Qx, Qy, Qz, Qw int32
	Mx, My, Mz int16
	Ts, Tsm uint32
	X_accel, Y_accel, Z_accel, X_mag, Y_mag, Z_mag int16
}

type MPUListener struct {
	r   *room
	//mpu mpu.IMUData
	mpu *IMUData
}

func NewMPUListener(r *room) MPUListener {
	ml := MPUListener{}
	ml.r = r

	log.Println("Initializing MPU")
	//mpu.InitMPU(10, 0)
	log.Println("MPU initialized")
	//defer mpu.CloseMPU()
	return ml
}

func (ml *MPUListener) run() {
	ml.mpu = new(IMUData)
	for {
		var err error
		//ml.mpu, err = mpu.ReadMPUAll()
		if err != nil {
			log.Println("Error reading mpu")
		}

		log.Println("Populating an IMUData")
		ml.mpu.Pitch += 0.0*rand.Float32()
		ml.mpu.Roll -= 0.4*rand.Float32()
		ml.mpu.Heading += rand.Float32()
		ml.mpu.Gx = int16(rand.Int())
		ml.mpu.Gy = int16(rand.Int())
		ml.mpu.Gz = int16(rand.Int())
		ml.mpu.Ax = int16(rand.Int())
		ml.mpu.Ay = int16(rand.Int())
		ml.mpu.Az = int16(rand.Int())
		ml.mpu.Qw = rand.Int31()
		ml.mpu.Qx = rand.Int31()
		ml.mpu.Qy = rand.Int31()
		ml.mpu.Qz = rand.Int31()
		ml.mpu.Mx = int16(rand.Int())
		ml.mpu.My = int16(rand.Int())
		ml.mpu.Mz = int16(rand.Int())
		ml.mpu.Ts = rand.Uint32()
		ml.mpu.Tsm = rand.Uint32()
		ml.mpu.X_accel = int16(rand.Int())
		ml.mpu.Y_accel = int16(rand.Int())
		ml.mpu.Z_accel = int16(rand.Int())
		ml.mpu.X_mag = int16(rand.Int())
		ml.mpu.Y_mag = int16(rand.Int())
		ml.mpu.Z_mag = int16(rand.Int())


		log.Println("Sending IMUData to forward channel")
		msg, err := json.Marshal(ml.mpu)
		if err != nil {
			log.Println("Couldn't marshall json data")
			continue
		}
		ml.r.forward <- msg

		time.Sleep(time.Duration(100)*time.Millisecond)
	}
}
