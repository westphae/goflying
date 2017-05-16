package main

import (
	"../../bmp280"
	"fmt"
	"github.com/kidoman/embd"
	"log"
	"math"
	"testing"
)

func TestBMP280Math(t *testing.T) {
	bmp := bmp280.BMP280{
		DigT: map[int]int32{
			1: 27504,
			2: 26435,
			3: -1000,
		},
		DigP: map[int]int64{
			1: 36477,
			2: -10685,
			3: 3024,
			4: 2855,
			5: 140,
			6: -7,
			7: 15500,
			8: -14600,
			9: 6000,
		},
	}

	raw_temp := int32(519888)
	raw_press := int64(415148)
	temp := bmp.CalcCompensatedTemp(raw_temp)
	press := bmp.CalcCompensatedPress(raw_press)

	t_fine := int32(128422)
	if bmp.T_fine != t_fine {
		log.Printf("t_fine mismatch: calculated %d, should be %d\n", bmp.T_fine, t_fine)
		t.Fail()
	} else {
		log.Printf("t_fine matched:  calculated %d, should be %d\n", bmp.T_fine, t_fine)
	}

	calc_temp := 25.08
	if math.Abs(temp-calc_temp) > 0.01 {
		log.Printf("temp mismatch:   calculated %f, should be %f\n", temp, calc_temp)
		t.Fail()
	} else {
		log.Printf("temp matched:    calculated %f, should be %f\n", temp, calc_temp)
	}

	calc_press := 1006.5327
	if math.Abs(press-calc_press) > 0.001 {
		log.Printf("press mismatch:  calculated %f, should be %f\n", press, calc_press)
		t.Fail()
	} else {
		log.Printf("press matched:   calculated %f, should be %f\n", press, calc_press)
	}

}

func TestBMP280Setup(t *testing.T) {
	const (
		mode          = bmp280.NormalMode
		standbyTime   = bmp280.StandbyTime250ms
		filterCoeff   = bmp280.FilterCoeff8
		oversampTemp  = bmp280.Oversamp16x
		oversampPress = bmp280.Oversamp16x
	)

	var (
		modes = []byte{
			bmp280.SleepMode,
			//bmp280.ForcedMode,
			bmp280.NormalMode,
		}
		standbyTimes = []byte{
			bmp280.StandbyTime1ms,
			bmp280.StandbyTime63ms,
			bmp280.StandbyTime125ms,
			bmp280.StandbyTime250ms,
			bmp280.StandbyTime500ms,
			bmp280.StandbyTime1000ms,
			bmp280.StandbyTime2000ms,
			bmp280.StandbyTime4000ms,
		}
		filterCoeffs = []byte{
			bmp280.FilterCoeffOff,
			bmp280.FilterCoeff2,
			bmp280.FilterCoeff4,
			bmp280.FilterCoeff8,
			bmp280.FilterCoeff16,
		}
		oversamps = []byte{
			bmp280.OversampSkipped,
			bmp280.Oversamp1x,
			bmp280.Oversamp2x,
			bmp280.Oversamp4x,
			bmp280.Oversamp8x,
			bmp280.Oversamp16x,
		}
		bmp *bmp280.BMP280
		err error
	)

	var checkAll = func(newMode, newStandbyTime, newFilterCoeff, newOversampTemp, newOversampPress byte) {
		var (
			curMode, curStandbyTime, curFilterCoeff, curOversampTemp, curOversampPress byte
			err                                                                        error
		)
		curMode, err = bmp.GetPowerMode()
		if err != nil {
			fmt.Printf("Error getting power mode: %s\n", err)
		}
		if curMode != newMode {
			fmt.Printf("Mode not set correctly: got %x, should be %x\n", curMode, newMode)
			t.Fail()
		}
		curStandbyTime, err = bmp.GetStandbyTime()
		if err != nil {
			fmt.Printf("Error getting standby time: %s\n", err)
		}
		if curStandbyTime != newStandbyTime {
			fmt.Printf("Standby time not set correctly: got %x, should be %x\n", curStandbyTime, newStandbyTime)
			t.Fail()
		}
		curFilterCoeff, err = bmp.GetFilterCoeff()
		if err != nil {
			fmt.Printf("Error getting filter coefficient: %s\n", err)
		}
		if curFilterCoeff != newFilterCoeff {
			fmt.Printf("Filter coefficient not set correctly: got %x, should be %x\n", curFilterCoeff, newFilterCoeff)
			t.Fail()
		}
		curOversampTemp, err = bmp.GetOversampTemp()
		if err != nil {
			fmt.Printf("Error getting temperature oversampling: %s\n", err)
		}
		if curOversampTemp != newOversampTemp {
			fmt.Printf("Temperature oversampling not set correctly: got %x, should be %x\n", curOversampTemp, newOversampTemp)
			t.Fail()
		}
		curOversampPress, err = bmp.GetOversampPress()
		if err != nil {
			fmt.Printf("Error getting pressure oversampling: %s\n", err)
		}
		if curOversampPress != newOversampPress {
			fmt.Printf("Pressure oversampling not set correctly: got %x, should be %x\n", curOversampPress, newOversampPress)
			t.Fail()
		}
	}

	i2cbus := embd.NewI2CBus(1)
	bmp, err = bmp280.NewBMP280(&i2cbus, bmp280.Address1, mode, standbyTime, filterCoeff, oversampTemp, oversampPress)
	if err != nil {
		bmp, err = bmp280.NewBMP280(i2cbus, bmp280.Address2, mode, standbyTime, filterCoeff, oversampTemp, oversampPress)
	}
	if err != nil {
		log.Println("Couldn't find a BMP280")
		t.Fail()
		return
	}

	// Test all initial gets
	fmt.Println("Checking initial setup values")
	checkAll(mode, standbyTime, filterCoeff, oversampTemp, oversampPress)

	// Test changing modes
	fmt.Println("\nChecking mode changes")
	for _, x := range modes {
		fmt.Printf("Setting mode to %x\n", x)
		err = bmp.SetPowerMode(x)
		if err != nil {
			fmt.Printf("Error setting power mode to %x: %s\n", x, err)
			t.Fail()
			continue
		}
		checkAll(x, standbyTime, filterCoeff, oversampTemp, oversampPress)
	}
	bmp.SetPowerMode(mode)

	// Test changing standby times
	fmt.Println("\nChecking standby time changes")
	for _, x := range standbyTimes {
		fmt.Printf("Setting standby time to %x\n", x)
		err = bmp.SetStandbyTime(x)
		if err != nil {
			fmt.Printf("Error setting standby time to %x: %s\n", x, err)
			t.Fail()
			continue
		}
		checkAll(mode, x, filterCoeff, oversampTemp, oversampPress)
	}
	bmp.SetStandbyTime(standbyTime)

	// Test changing filter coefficients
	fmt.Println("\nChecking filter coefficient changes")
	for _, x := range filterCoeffs {
		fmt.Printf("Setting filter coefficient to %x\n", x)
		err = bmp.SetFilterCoeff(x)
		if err != nil {
			fmt.Printf("Error setting filter coefficient to %x: %s\n", x, err)
			t.Fail()
			continue
		}
		checkAll(mode, standbyTime, x, oversampTemp, oversampPress)
	}
	bmp.SetFilterCoeff(filterCoeff)

	// Test changing temperature oversampling
	fmt.Println("\nChecking temperature oversampling changes")
	for _, x := range oversamps {
		fmt.Printf("Setting temperature oversampling to %x\n", x)
		err = bmp.SetOversampTemp(x)
		if err != nil {
			fmt.Printf("Error setting temperature oversampling to %x: %s\n", x, err)
			t.Fail()
			continue
		}
		checkAll(mode, standbyTime, filterCoeff, x, oversampPress)
	}
	bmp.SetOversampTemp(oversampTemp)

	// Test changing pressure oversampling
	fmt.Println("\nChecking pressure oversampling changes")
	for _, x := range oversamps {
		fmt.Printf("Setting pressure oversampling to %x\n", x)
		err = bmp.SetOversampPress(x)
		if err != nil {
			fmt.Printf("Error setting pressure oversampling to %x: %s\n", x, err)
			t.Fail()
			continue
		}
		checkAll(mode, standbyTime, filterCoeff, oversampTemp, x)
	}
	bmp.SetOversampPress(oversampPress)
}
