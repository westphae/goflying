package main

import (
	"fmt"
	"log"
	//"github.com/westphae/linux-mpu9150/mpu"
	"math/rand"
	"time"
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
	for {
		var err error
		//ml.mpu, err = mpu.ReadMPUAll()
		if err != nil {
			log.Println("Error reading mpu")
		}

		log.Println("Populating an IMUData")
		ml.mpu = new(IMUData)
		ml.mpu.Pitch = rand.Float32()
		ml.mpu.Roll = rand.Float32()
		ml.mpu.Heading = rand.Float32()
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
		ml.r.forward <- []byte(fmt.Sprintf("%f,%f,%f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
			ml.mpu.Pitch, ml.mpu.Roll, ml.mpu.Heading,
			ml.mpu.Gx, ml.mpu.Gy, ml.mpu.Gz, ml.mpu.Ax, ml.mpu.Ay, ml.mpu.Az,
			ml.mpu.Qx, ml.mpu.Qy, ml.mpu.Qz, ml.mpu.Qw,
			ml.mpu.Mx, ml.mpu.My, ml.mpu.Mz,
			ml.mpu.Ts, ml.mpu.Tsm,
			ml.mpu.X_accel, ml.mpu.Y_accel, ml.mpu.Z_accel, ml.mpu.X_mag, ml.mpu.Y_mag, ml.mpu.Z_mag))
		//log.Printf("T: %d\n", ml.mpu.Ts)
		time.Sleep(5*time.Second)
	}
}
