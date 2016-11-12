package main

import (
	"testing"
	"github.com/westphae/goflying/bmp280"
	"log"
	"math"
)

func TestBMP280Math(t *testing.T) {
	bmp := bmp280.BMP280{
		DigT1: 27504,
		DigT2: 26435,
		DigT3: -1000,
		DigP1: 36477,
		DigP2: -10685,
		DigP3: 3024,
		DigP4: 2855,
		DigP5:140,
		DigP6: -7,
		DigP7: 15500,
		DigP8: -14600,
		DigP9: 6000,
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
	if math.Abs(temp - calc_temp) > 0.01 {
		log.Printf("temp mismatch:   calculated %f, should be %f\n", temp, calc_temp)
		t.Fail()
	} else {
		log.Printf("temp matched:    calculated %f, should be %f\n", temp, calc_temp)
	}

	calc_press := 1006.5327
	if math.Abs(press - calc_press) > 0.001 {
		log.Printf("press mismatch:  calculated %f, should be %f\n", press, calc_press)
		t.Fail()
	} else {
		log.Printf("press matched:   calculated %f, should be %f\n", press, calc_press)
	}

}

