package sensors

import (
	"encoding/json"
	"fmt"
	"log"
	"os"
	"time"
)

const calDataLocation = "/etc/imu_cal.json"

type PressureSensor struct {
	C    <-chan *BMPData
	CBuf <-chan *BMPData
}

type IMUSensor struct {
	C    <-chan *IMUData // Current instantaneous sensor values
	CAvg <-chan *IMUData // Average sensor values (since CAvg last read)
	CBuf <-chan *IMUData // Buffer of instantaneous sensor values
}

type BMPData struct {
	Temperature float64
	Pressure    float64
	T           time.Duration
}

// IMUData contains all the values measured by an MPU9250, ICM20948 or equivalent.
type IMUData struct {
	G1, G2, G3        float64
	A1, A2, A3        float64
	M1, M2, M3        float64
	Temp              float64
	GAError, MagError error
	N, NM             int
	T, TM             time.Time
	DT, DTM           time.Duration
}

type IMUCalData struct {
	A01, A02, A03    float64 // Accelerometer hardware bias
	G01, G02, G03    float64 // Gyro hardware bias
	M01, M02, M03    float64 // Magnetometer hardware bias
	Ms11, Ms12, Ms13 float64 // Magnetometer rescaling matrix
	Ms21, Ms22, Ms23 float64 // (Only diagonal is used currently)
	Ms31, Ms32, Ms33 float64
}

func (d *IMUCalData) Reset() {
	d.Ms11 = 1
	d.Ms22 = 1
	d.Ms33 = 1
}

func (d *IMUCalData) Save() {
	fd, err := os.OpenFile(calDataLocation, os.O_CREATE|os.O_WRONLY|os.O_TRUNC, os.FileMode(0644))
	if err != nil {
		log.Printf("error saving imu calibration data to %s: %s", calDataLocation, err.Error())
		return
	}
	defer fd.Close()
	calData, err := json.Marshal(d)
	if err != nil {
		log.Printf("error marshaling imu calibration data: %s", err)
		return
	}
	fd.Write(calData)
}

func (d *IMUCalData) Load() (err error) {
	errstr := "error reading imu calibration data from %s: %s"
	fd, rerr := os.Open(calDataLocation)
	if rerr != nil {
		err = fmt.Errorf(errstr, calDataLocation, rerr.Error())
		return
	}
	defer fd.Close()
	buf := make([]byte, 1024)
	count, rerr := fd.Read(buf)
	if rerr != nil {
		err = fmt.Errorf(errstr, calDataLocation, rerr.Error())
		return
	}
	rerr = json.Unmarshal(buf[0:count], d)
	if rerr != nil {
		err = fmt.Errorf(errstr, calDataLocation, rerr.Error())
		return
	}
	return
}
