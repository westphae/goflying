package main

import (
	"fmt"
	"time"

	"../../ahrs"
	"../../mpu9250"
)

func main() {
	var (
		mpu    *mpu9250.MPU9250
		cur    *mpu9250.MPUData
		err    error
		t0     time.Time
		logMap map[string]interface{} // Map only for analysis/debugging
	)

	for i := 0; i < 10; i++ {
		mpu, err = mpu9250.NewMPU9250(250, 4, 1000, true, false)
		if err != nil {
			fmt.Printf("Error initializing MPU9250, attempt %d of 10\n", i)
			time.Sleep(5 * time.Second)
		} else {
			break
		}
	}

	if err != nil {
		fmt.Println("Error: couldn't initialize MPU9250")
		return
	} else {
		fmt.Println("MPU9250 initialized successfully")
	}

	/*
		mpu.CCal<- 1
		fmt.Println("Awaiting Calibration Result")
		if err := <-mpu.CCalResult; err != nil {
			fmt.Println(err.Error())
			return
		} else {
			fmt.Println("Calibration succeeded")
		}
	*/

	t0 = time.Now()
	logMap = make(map[string]interface{})
	updateLogMap(t0, new(mpu9250.MPUData), logMap)
	filename := fmt.Sprintf("/var/log/mpudata_%s.csv", time.Now().Format("20060102_150405"))
	logger := ahrs.NewAHRSLogger(filename, logMap)
	defer logger.Close()

	fmt.Printf("Recording data log to %s\n", filename)
	defer fmt.Println("Finished recording data log.")
	for {
		cur = <-mpu.CBuf
		updateLogMap(t0, cur, logMap)
		logger.Log()
	}
}

var sensorLogMap = map[string]func(t0 time.Time, m *mpu9250.MPUData) float64{
	"T":    func(t0 time.Time, m *mpu9250.MPUData) float64 { return float64(m.T.Sub(t0).Nanoseconds()/1000) / 1000 },
	"TM":   func(t0 time.Time, m *mpu9250.MPUData) float64 { return float64(m.TM.Sub(t0).Nanoseconds()/1000) / 1000 },
	"A1":   func(t0 time.Time, m *mpu9250.MPUData) float64 { return m.A1 },
	"A2":   func(t0 time.Time, m *mpu9250.MPUData) float64 { return m.A2 },
	"A3":   func(t0 time.Time, m *mpu9250.MPUData) float64 { return m.A3 },
	"B1":   func(t0 time.Time, m *mpu9250.MPUData) float64 { return m.G1 },
	"B2":   func(t0 time.Time, m *mpu9250.MPUData) float64 { return m.G2 },
	"B3":   func(t0 time.Time, m *mpu9250.MPUData) float64 { return m.G3 },
	"M1":   func(t0 time.Time, m *mpu9250.MPUData) float64 { return m.M1 },
	"M2":   func(t0 time.Time, m *mpu9250.MPUData) float64 { return m.M2 },
	"M3":   func(t0 time.Time, m *mpu9250.MPUData) float64 { return m.M3 },
	"Temp": func(t0 time.Time, m *mpu9250.MPUData) float64 { return m.Temp },
}

func updateLogMap(t0 time.Time, m *mpu9250.MPUData, p map[string]interface{}) {
	for k := range sensorLogMap {
		p[k] = sensorLogMap[k](t0, m)
	}
}
