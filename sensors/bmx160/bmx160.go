package bmx160

import (
	"errors"
	"fmt"
	"log"
	"math"
	"time"
	"github.com/kidoman/embd"
	_ "github.com/kidoman/embd/host/all" // Empty import needed to initialize embd library.
	_ "github.com/kidoman/embd/host/rpi" // Empty import needed to initialize embd library.
)

const (
	bufSize         = 25 // Size of buffer storing instantaneous sensor values
	scaleMag        = 9830.0 / 65536
	calDataLocation = "/etc/bmx160cal.json"
)


//contains the trim values need to calculate the magnetic field
type bmm150_trim_registers struct
{
    /*! trim x1 data */
    dig_x1 byte

    /*! trim y1 data */
    dig_y1 byte 

    /*! trim x2 data */
    dig_x2 byte

    /*! trim y2 data */
    dig_y2 byte

    /*! trim z1 data */
    dig_z1 uint16

    /*! trim z2 data */
    dig_z2 int16

    /*! trim z3 data */
    dig_z3 int16

    /*! trim z4 data */
    dig_z4 int16

    /*! trim xy1 data */
    dig_xy1 byte

    /*! trim xy2 data */
    dig_xy2 byte

    /*! trim xyz1 data */
    dig_xyz1 uint16
}


// MPUData contains all the values measured by an BMX160.
type MPUData struct {
	G1, G2, G3        float64
	A1, A2, A3        float64
	M1, M2, M3        float64
	Temp              float64
	GAError, MagError error
	N, NM             int
	T, TM             time.Time
	DT, DTM           time.Duration
}


/*
BMX160 represents a Bosch BMX160 9DoF chip.
communication is via channels.
*/
type BMX160 struct {
	i2cbus                embd.I2CBus
	scaleGyro, scaleAccel float64 // Max sensor reading for value 2**15-1
	sampleRate            int
	enableMag             bool
        magnSensTrimValues    bmm150_trim_registers
	mcal1, mcal2, mcal3 float64         // Hardware magnetometer calibration values, uT
	C                   <-chan *MPUData // Current instantaneous sensor values
	CAvg                <-chan *MPUData // Average sensor values (since CAvg last read)
	CBuf                <-chan *MPUData // Buffer of instantaneous sensor values
	cClose              chan bool       // Turn off MPU polling
}

/*
NewBMX160 creates a new BMX160 object according to the supplied parameters.  If there is no BMX160 available or there
is an error creating the object, an error is returned.
*/
//ToDo Selftest of sensors
//temperature corrections
//magnetometer is not used
func NewBMX160(i2cbus *embd.I2CBus, sensitivityGyro, sensitivityAccel, sampleRate int, enableMag bool, applyHWOffsets bool) (*BMX160, error) {
	var mpu = new(BMX160)
	mpu.i2cbus = *i2cbus
	mpu.sampleRate = sampleRate
	mpu.enableMag = enableMag
        
        id, err := mpu.GetChipID()
        if id != 0xd8 {
           log.Printf("chip is not BMX160! Chip ID is %x\n", id)
           return nil, errors.New("chip is not BMX160!")
	}
        log.Printf("found IMU chip BMX160!\n")

        if id != BMI160_CHIP_ID {
                return nil, errors.New("IMU is not BMX160")
        }
       
        //selftest of the IMU
        //Testing the accelerometer
        //switching to normal power mode
        if err := mpu.i2cWrite(BMI160_COMMAND_REG_ADDR,BMI160_ACCEL_NORMAL_MODE); err != nil { 
               return nil, errors.New("Error setting accelerometer in normal mode")
        }
        time.Sleep(5 * time.Millisecond) // wait time to switch to normal mode
          
       //configuring the accelerometer for the test per datasheet instructions
       if err := mpu.i2cWrite(BMI160_ACCEL_CONFIG_ADDR,0x2C); err != nil { 
               return nil, errors.New("Error setting up for accelerometer test")
        }
        time.Sleep(5 * time.Millisecond) // wait time to switch to normal mode
          
        if err := mpu.i2cWrite(BMI160_ACCEL_RANGE_ADDR,0x08); err != nil { 
               return nil, errors.New("Error setting up for accelerometer test")
        }
        time.Sleep(5 * time.Millisecond) // wait time to switch to normal mode

        //activating the test 
        if err := mpu.i2cWrite(BMI160_SELF_TEST_ADDR,0x09); err != nil { 
               return nil, errors.New("Error setting up for accelerometer test")
        }
        time.Sleep(60 * time.Millisecond) // wait time to switch to normal mode
	p, gaError := mpu.i2cRead2(BMI160_ACCEL_DATA_ADDR)
	  if gaError != nil {
		log.Println("BMX160 Warning: error reading gyro/accel")
	  }
        testAccX := float64(p)*8.0 / float64(math.MaxInt16)

	p, gaError = mpu.i2cRead2(BMI160_ACCEL_DATA_ADDR+0x02)
	  if gaError != nil {
		log.Println("BMX160 Warning: error reading gyro/accel")
	  }
        testAccY := float64(p)*8.0 / float64(math.MaxInt16)

	p, gaError = mpu.i2cRead2(BMI160_ACCEL_DATA_ADDR+0x04)
	  if gaError != nil {
		log.Println("BMX160 Warning: error reading gyro/accel")
	  }
        testAccZ := float64(p)*8.0 / float64(math.MaxInt16)

        //activating the test in the opposite direction
        if err := mpu.i2cWrite(BMI160_SELF_TEST_ADDR,0x0D); err != nil { 
               return nil, errors.New("Error setting up for accelerometer test")
        }
        time.Sleep(60 * time.Millisecond) // wait time to switch to normal mode

	p, gaError = mpu.i2cRead2(BMI160_ACCEL_DATA_ADDR)
	  if gaError != nil {
		log.Println("BMX160 Warning: error reading gyro/accel")
	  }
        testAccX -= float64(p)*8.0 / float64(math.MaxInt16)
        log.Printf("BMX160: X-axis accelerometer self test value is: %f (needs to be >2)\n",testAccX)
        if testAccX<2 { 
               return nil, errors.New("BMX160 IMU selftest failed for X-axis accelerometer")
        }

	p, gaError = mpu.i2cRead2(BMI160_ACCEL_DATA_ADDR+0x02)
	  if gaError != nil {
		log.Println("BMX160 Warning: error reading gyro/accel")
	  }
        testAccY -= float64(p)*8.0 / float64(math.MaxInt16)
        log.Printf("BMX160: Y-axis accelerometer self test value is: %f (needs to be >2)\n",testAccY)
        if testAccY<2 { 
               return nil, errors.New("BMX160 IMU selftest failed for Y-axis accelerometer")
        }

	p, gaError = mpu.i2cRead2(BMI160_ACCEL_DATA_ADDR+0x04)
	  if gaError != nil {
		log.Println("BMX160 Warning: error reading gyro/accel")
	  }
        testAccZ -= float64(p)*8.0 / float64(math.MaxInt16)
        log.Printf("BMX160: Z-axis accelerometer self test value is: %f (needs to be >2)\n",testAccZ)
        if testAccZ<2 { 
               return nil, errors.New("BMX160 IMU selftest failed for Z-axis accelerometer")
        }

        //Testing the gyro
        //setting it in normal mode
        if err := mpu.i2cWrite(BMI160_COMMAND_REG_ADDR,BMI160_GYRO_NORMAL_MODE); err != nil { 
               return nil, errors.New("Error setting gyroscope into normal mode")
        }
        time.Sleep(90 * time.Millisecond) // wait time to switch to mormal mode
        //activating the gyro test
        if err := mpu.i2cWrite(BMI160_SELF_TEST_ADDR,0x10); err != nil { 
               return nil, errors.New("Error setting up for gyro test")
        }
        time.Sleep(20 * time.Millisecond) // wait time to let test finish

        //getting the gyro test result
	stat, err := mpu.i2cRead(BMI160_STATUS_ADDR)
	if err != nil {
		return nil,errors.New("BMX160 Error: Reading Status Byte after gyro test")
	}

        if stat&BMI160_GYRO_SELF_TEST_STATUS_MSK != 0x02 {
		return nil,errors.New("BMX160 Error: gyro selftest failed")
        }
	log.Println("BMX160 gyro passed selftest")
	log.Println("BMX160 all IMU tests completed and passed")
        //selftest of the IMU completed
 
        //soft reset of IMU
        if err := mpu.i2cWrite(BMI160_COMMAND_REG_ADDR,BMI160_SOFT_RESET_CMD); err != nil { 
               return nil, errors.New("Error resetting BMX160")
        }


        //set all instruments: accelerometer, gyroscope, and magnetometer into normal power mode
        if err := mpu.i2cWrite(BMI160_COMMAND_REG_ADDR,BMI160_ACCEL_NORMAL_MODE); err != nil { 
               return nil, errors.New("Error setting accelerometer in normal mode")
        }
        time.Sleep(5 * time.Millisecond) // wait time to switch to normal mode


        if err := mpu.i2cWrite(BMI160_COMMAND_REG_ADDR,BMI160_GYRO_NORMAL_MODE); err != nil { 
               return nil, errors.New("Error setting gyroscope into normal mode")
        }
        time.Sleep(90 * time.Millisecond) // wait time to switch to mormal mode

	value, err := mpu.i2cRead(BMI160_PMU_STATUS_ADDR)
	if err != nil {
		return nil, errors.New("Error reading BMX160 power mode status")
	}


        //configuring gyro
	if err := mpu.SetSampleRateGyro(mpu.sampleRate); err != nil {
		return nil, err
	}

	if err := mpu.SetGyroSensitivity(sensitivityGyro); err != nil {
		log.Println(err)
	}


        //configuring accelerometer
	if err := mpu.SetSampleRateAccel(mpu.sampleRate); err != nil {
		return nil, err
	}

	if err := mpu.SetAccelSensitivity(sensitivityAccel); err != nil {
		log.Println(err)
	}
        

        //Turning FIFO off
	if err := mpu.i2cWrite(BMI160_FIFO_CONFIG_0_ADDR, 0x80); err != nil {
		return nil, errors.New("BMX160 Error: couldn't disable FIFO")
	}
	value, err = mpu.i2cRead(BMI160_FIFO_CONFIG_0_ADDR)
        if  err != nil {
		return nil, errors.New("Error reading BMX160 FIFO Config address 0")
	}
        if value != 0x80 {
                return nil, errors.New("BMX160 FIFO ADDR 0 is not configured in the default way")
        }

	if err := mpu.i2cWrite(BMI160_FIFO_CONFIG_1_ADDR, 0x10); err != nil {
		return nil, errors.New("BMX160 Error: couldn't disable FIFO")
	}
	value, err = mpu.i2cRead(BMI160_FIFO_CONFIG_1_ADDR)
        if  err != nil {
		return nil, errors.New("Error reading BMX160 FIFO Config address 0")
	}
        if value != 0x10 {
                return nil, errors.New("BMX160 FIFO ADDR 1 is not configure in the default way")
        }
        


	// Turn off interrupts
	if err := mpu.i2cWrite(BMI160_INT_ENABLE_0_ADDR, 0x00); err != nil {
		return nil, errors.New("BMX160 Error: couldn't disable interrupts")
	}
	if err := mpu.i2cWrite(BMI160_INT_ENABLE_1_ADDR, 0x00); err != nil {
		return nil, errors.New("BMX160 Error: couldn't disable interrupts")
	}
	if err := mpu.i2cWrite(BMI160_INT_ENABLE_2_ADDR, 0x00); err != nil {
		return nil, errors.New("BMX160 Error: couldn't disable interrupts")
	}

        if applyHWOffsets == true {
            mpu.FastOffsetCompensation()
            mpu.EnableOffsetCompensation()
        }

        ////////////////////////////////////////////////////////////////////////
        // Magnetometer (non-functional)
	// Set up magnetometer
	if mpu.enableMag {
    
           if err := mpu.i2cWrite(BMI160_COMMAND_REG_ADDR,BMI160_AUX_NORMAL_MODE); err != nil { 
               return nil, errors.New("Error setting magnetometer into normal mode")
           }
           time.Sleep(50 * time.Millisecond) // wait time to switch to normal mode


           //enable the secondary interface
           //per BMI160 data sheet bit 5 and 4
           //00: primary interface: autoconfig / secondary interface: off
           //01: Primary interface:I2C / secondary interface:OIS
           //02: Primary interface: autoconfig / secondary interface: Magnetometer
           //11: reserved
           if_conf, err := mpu.i2cRead(BMI160_IF_CONF_ADDR)
           if  err != nil {
                return nil, errors.New("Error reading BMX160 BMI160_IF_CONF_ADDR")
           }

           if_conf |= byte(1 << 5);

           if err := mpu.i2cWrite(BMI160_IF_CONF_ADDR, if_conf); err != nil {
		return nil, errors.New("BMX160 Error: couldn't enable aux interface")
           }


          if err := mpu.i2cWrite(BMI160_AUX_IF_1_ADDR, BMI160_MANUAL_MODE_EN_MSK); err != nil {
               return nil, errors.New("BMX160 Error: couldn't set magnetic sensor in normal mode")
          }
          time.Sleep(50*time.Millisecond) // wait for command to trickle through 


          /////////////////////////////////////////////////////////////////////////////// 


          var aux_if byte

          //Get the configuration of the interface: shows 0x80 -> only bit 7 is enabled 
          //enables magnetometer register access, mag data are not updated!
          //max mag_offset is maximal 2.5ms.
          //read burst operation reads only one byte at a time 
          aux_if, err = mpu.i2cRead(BMI160_AUX_IF_1_ADDR)
          fmt.Printf("Config of magn interface in BMI160 AUX_IF_1_ADDR or MAG_IF_1 %x\n", aux_if)
          if  err != nil {
                return nil, errors.New("Error reading ")
          }
          time.Sleep(time.Millisecond)


          //from here on we try to talk with the BMM150 magnetic sensor that is 
          //internally connected inside the BMX160 to the IMU (it doesn't work, no response from sensor)
          //turn magnetic sensor power on
          var reg_data = make([]byte, 1)
          if err:= mpu.bmm150_get_regs(BMM150_POWER_CONTROL_ADDR, reg_data); err != nil {
               return nil, errors.New("BMX160 Error: couldn't disable interrupts")
          }

          reg_data[0] =  (reg_data[0] & ^((byte)(BMM150_PWR_CNTRL_MSK))) | BMM150_POWER_CNTRL_ENABLE
          fmt.Printf("what we want it to set to %x\n", reg_data[0])

          if err := mpu.bmm150_set_regs(BMM150_POWER_CONTROL_ADDR,reg_data[0]); err != nil {
                return nil, errors.New("BMX160 Error: couldn't disable interrupts")
          }

          /* Start-up time delay of 3ms*/
          time.Sleep(BMM150_START_UP_TIME*time.Millisecond) 
          if err:= mpu.bmm150_get_regs(BMM150_POWER_CONTROL_ADDR, reg_data); err != nil {
               return nil, errors.New("BMX160 Error: Couldn't retrieve magnetometer read register")
          }
          fmt.Printf("what it is %x\n", reg_data[0])

          //self test
          /*
          if err:= mpu.bmm150_get_regs(0x4C, reg_data); err != nil {
             return nil, errors.New("BMX160 Error: Couldn't retrieve magnetometer read register")
          }
          fmt.Printf("bmm150 op add %x\n", reg_data[0])
          if err := mpu.bmm150_set_regs(0x4C,0x01); err != nil {
               return nil, errors.New("BMX160 Error: Couldn't retrieve magnetometer read register")
          }
          if err:= mpu.bmm150_get_regs(0x4C, reg_data); err != nil {
             return nil, errors.New("BMX160 Error: Couldn't retrieve magnetometer read register")
          }
          fmt.Printf("bmm150 op add %x\n", reg_data[0])

          //test result X  
          if err:= mpu.bmm150_get_regs(0x42, reg_data); err != nil {
             return nil, errors.New("BMX160 Error: Couldn't retrieve magnetometer read register")
          }
          fmt.Printf("self test result X %x\n", reg_data[0])

          //test result Y  
          if err:= mpu.bmm150_get_regs(0x44, reg_data); err != nil {
             return nil, errors.New("BMX160 Error: Couldn't retrieve magnetometer read register")
          }
          fmt.Printf("self test result Y %x\n", reg_data[0])

          //test result Z  
          if err:= mpu.bmm150_get_regs(0x46, reg_data); err != nil {
                return nil, errors.New("BMX160 Error: Couldn't retrieve magnetometer read register")
          }
          fmt.Printf("self test result Z %x\n", reg_data[0])
          */

          /* Chip ID of the sensor is read */
          var chip_id byte
          if err:= mpu.bmm150_get_regs(BMM150_CHIP_ID_ADDR, reg_data); err != nil {
             return nil, errors.New("BMX160 Error: Couldn't retrieve magnetometer read register")
          }
          fmt.Printf("bmm chip id %x\n", chip_id)

          /* Function to update trim values */
          mpu.read_trim_registers()

          //set read burst to 8 bytes
          aux_conf, err := mpu.i2cRead(BMI160_AUX_IF_1_ADDR)
          fmt.Printf("BMI160_AUX_IF_1_ADDR %x\n", aux_conf)
          if  err != nil {
              return nil, errors.New("Error reading ")
          }

          aux_conf = (aux_conf | 0x03 )
          fmt.Printf("new BMI160_AUX_IF_1_ADDR %x\n", aux_conf)
          if err := mpu.i2cWrite(BMI160_AUX_IF_1_ADDR, aux_conf); err != nil {
   	     return nil, errors.New("BMX160 Error: couldn't write to AUX IF")
          }
          time.Sleep(time.Millisecond) // wait for command to trickle through 
  
          /////////////////////////////////////

              //set regular precision XY
	      if err := mpu.i2cWrite(BMI160_AUX_IF_4_ADDR, 0x04); err != nil {
	  	return nil, errors.New("BMX160 Error: couldn't disable interrupts")
	      }
	      if err := mpu.i2cWrite(BMI160_AUX_IF_3_ADDR, 0x51); err != nil {
	  	return nil, errors.New("BMX160 Error: couldn't disable interrupts")
	      }
              time.Sleep(time.Millisecond) // wait for command to trickle through 

              //set regular precision Z
	      if err := mpu.i2cWrite(BMI160_AUX_IF_4_ADDR, 0x0E); err != nil {
	  	return nil, errors.New("BMX160 Error: couldn't disable interrupts")
	      }
	      if err := mpu.i2cWrite(BMI160_AUX_IF_3_ADDR, 0x52); err != nil {
	  	return nil, errors.New("BMX160 Error: couldn't disable interrupts")
	      }
              time.Sleep(time.Millisecond) // wait for command to trickle through 

              //ODR for magnetic sensor
	      if err := mpu.i2cWrite(BMI160_AUX_ODR_ADDR,BMI160_AUX_ODR_12_5HZ); err != nil {
	  	return nil, errors.New("BMX160 Error: couldn't disable interrupts")
	      }

              //prepare for data mode
	      if err := mpu.i2cWrite(BMI160_AUX_IF_4_ADDR, 0x02); err != nil {
	  	return nil, errors.New("BMX160 Error: couldn't disable interrupts")
	      }
	      if err := mpu.i2cWrite(BMI160_AUX_IF_3_ADDR, 0x4C); err != nil {
	  	return nil, errors.New("BMX160 Error: couldn't disable interrupts")
	      }
              time.Sleep(time.Millisecond) // wait for command to trickle through 
	      if err := mpu.i2cWrite(BMI160_AUX_IF_2_ADDR, 0x42); err != nil {
	  	return nil, errors.New("BMX160 Error: couldn't disable interrupts")
	      }
              time.Sleep(time.Millisecond) // wait for command to trickle through 

              //set into data mode
	      if err := mpu.i2cWrite(BMI160_AUX_IF_1_ADDR,0x0); err != nil {
	  	return nil, errors.New("BMX160 Error: couldn't disable interrupts")
	      }
               
              //check if magnetic sensor is in normal power mode
	      value, err = mpu.i2cRead(BMI160_PMU_STATUS_ADDR)
              if  err != nil {
	  	  return nil, errors.New("Error reading BMX160 FIFO Config address 0")
	       }
               if value&0b11 != 0x01 {
                   return nil, errors.New("BMX160 Magnetfield sensor is not in normal mode")
               }

	       time.Sleep(100 * time.Millisecond) // Make sure mag is ready
               log.Printf("magnetic sensor setup complete\n")
	} //done with the magnetic sensor

	go mpu.readSensors()

	// Give the IMU time to fully initialize and then clear out any bad values from the averages.
	time.Sleep(500 * time.Millisecond) // Make sure it's ready
	<-mpu.CAvg

	return mpu, nil
}

// readSensors polls the gyro, accelerometer and magnetometer sensors as well as the die temperature.
// Communication is via channels.
func (mpu *BMX160) readSensors() {
	var (
		g1, g2, g3, a1, a2, a3, m1, m2, m3, tmp int16   // Current values
		avg1, avg2, avg3, ava1, ava2, ava3, avtmp   float64 // Accumulators for averages
		avm1, avm2, avm3                            int32
		n, nm                                       float64
		gaError, magError                           error
		t0, t, t0m, tm                              time.Time
		magSampleRate                               int
		curdata                                     *MPUData
	)

	acRegMap := map[*int16]byte{
		&g1: BMI160_GYRO_DATA_ADDR, &g2: BMI160_GYRO_DATA_ADDR+0x02, &g3: BMI160_GYRO_DATA_ADDR+0x04,
		&a1: BMI160_ACCEL_DATA_ADDR, &a2: BMI160_ACCEL_DATA_ADDR+0x02, &a3: BMI160_ACCEL_DATA_ADDR+0x04,
		&tmp: 0x20,
	}

	//magRegMap := map[*int16]byte{
	//	&m1: BMI160_AUX_DATA_ADDR, &m2: BMI160_AUX_DATA_ADDR+0x02, &m3: BMI160_AUX_DATA_ADDR+0x04, &m4: BMI160_AUX_DATA_ADDR+0x06,
	//}

	cC := make(chan *MPUData)
	defer close(cC)
	mpu.C = cC
	cAvg := make(chan *MPUData)
	defer close(cAvg)
	mpu.CAvg = cAvg
	cBuf := make(chan *MPUData, bufSize)
	defer close(cBuf)
	mpu.CBuf = cBuf
	mpu.cClose = make(chan bool)
	defer close(mpu.cClose)

	clock := time.NewTicker(time.Duration(int(10000.0/float32(mpu.sampleRate)+0.5)) * time.Millisecond)
	//TODO westphae: use the clock to record actual time instead of a timer
	defer clock.Stop()

	clockMag := time.NewTicker(time.Duration(int(1000.0/float32(magSampleRate)+0.5)) * time.Millisecond)
	t0 = time.Now()
	t0m = time.Now()


	makeMPUData := func() *MPUData {
		//mm1 := float64(m1)*mpu.mcal1 - mpu.M01
		//mm2 := float64(m2)*mpu.mcal2 - mpu.M02
		//mm3 := float64(m3)*mpu.mcal3 - mpu.M03
		d := MPUData{
			G1:      (float64(g1)) * mpu.scaleGyro,
			G2:      -1.0 * (float64(g2)) * mpu.scaleGyro,
			G3:      -1.0 * (float64(g3)) * mpu.scaleGyro,
			A1:      (float64(a1)) * mpu.scaleAccel,
			A2:      -1.0 * (float64(a2)) * mpu.scaleAccel,
			A3:      -1.0 * (float64(a3)) * mpu.scaleAccel,
			//M1:      mpu.Ms11*mm1 + mpu.Ms12*mm2 + mpu.Ms13*mm3,
			//M2:      mpu.Ms21*mm1 + mpu.Ms22*mm2 + mpu.Ms23*mm3,
			//M3:      mpu.Ms31*mm1 + mpu.Ms32*mm2 + mpu.Ms33*mm3,
			M1:      float64(m1),
			M2:      float64(m2),
			M3:      float64(m3),
			Temp:    float64(tmp)*0.001938 + 23.0,
			GAError: gaError, MagError: magError,
			N: 1, NM: 1,
			T: t, TM: tm,
			DT: time.Duration(0), DTM: time.Duration(0),
		}
		if gaError != nil {
			d.N = 0
		}
		if magError != nil {
			d.NM = 0
		}
		return &d
	}

	makeAvgMPUData := func() *MPUData {
		//mm1 := float64(avm1)*mpu.mcal1/nm - mpu.M01
		//mm2 := float64(avm2)*mpu.mcal2/nm - mpu.M02
		//mm3 := float64(avm3)*mpu.mcal3/nm - mpu.M03
		d := MPUData{}
		if n > 0.5 {
			d.G1 = (avg1/n) * mpu.scaleGyro
			d.G2 = -1.0 * (avg2/n) * mpu.scaleGyro
			d.G3 = -1.0 * (avg3/n) * mpu.scaleGyro
			d.A1 = (ava1/n) * mpu.scaleAccel
			d.A2 = -1.0 * (ava2/n) * mpu.scaleAccel
			d.A3 = -1.0 * (ava3/n) * mpu.scaleAccel
			d.Temp = (float64(avtmp)/n)*0.001938 + 23.0
			d.N = int(n + 0.5)
			d.T = t
			d.DT = t.Sub(t0)
		} else {
			d.GAError = errors.New("BMX160 Error: No new accel/gyro values")
		}
		if nm > 0 {
			//d.M1 = mpu.Ms11*mm1 + mpu.Ms12*mm2 + mpu.Ms13*mm3
			//d.M2 = mpu.Ms21*mm1 + mpu.Ms22*mm2 + mpu.Ms23*mm3
			//d.M3 = mpu.Ms31*mm1 + mpu.Ms32*mm2 + mpu.Ms33*mm3
			d.M1 = float64(avm1)/nm
			d.M2 = float64(avm2)/nm
			d.M3 = float64(avm3)/nm
			d.NM = int(nm + 0.5)
			d.TM = tm
			d.DTM = t.Sub(t0m)
		} else {
			d.MagError = errors.New("BMX160 Error: No new magnetometer values")
		}
		return &d
	}

	for {
		select {
		case t = <-clock.C: // Read accel/gyro data:
			for p, reg := range acRegMap {
				*p, gaError = mpu.i2cRead2(reg)
				if gaError != nil {
					log.Println("BMX160 Warning: error reading gyro/accel")
				}
			}

			curdata = makeMPUData()
			// Update accumulated values and increment count of gyro/accel readings
			avg1 += float64(g1)
			avg2 += float64(g2)
			avg3 += float64(g3)
			ava1 += float64(a1)
			ava2 += float64(a2)
			ava3 += float64(a3)
			avtmp += float64(tmp)
			avm1 += int32(m1)
			avm2 += int32(m2)
			avm3 += int32(m3)
			n++
			select {
			case cBuf <- curdata: // We update the buffer every time we read a new value.
			default: // If buffer is full, remove oldest value and put in newest.
				<-cBuf
				cBuf <- curdata
			}
		      case tm = <-clockMag.C: // Read magnetometer data:
                       //Below has not been adapted for the BMX 160 and thus doesn't work for the BMX160
                       //Need to fix getting access to the magentosensor first
			if mpu.enableMag {
			        //get the date from the magnetometer before proceeding to the next lines

				// Test validity of magnetometer data

				// Update values and increment count of magnetometer readings
				avm1 += int32(m1)
				avm2 += int32(m2)
				avm3 += int32(m3)
				nm++
			}
		case cC <- curdata: // Send the latest values
		case cAvg <- makeAvgMPUData(): // Send the averages
			avg1, avg2, avg3 = 0, 0, 0
			ava1, ava2, ava3 = 0, 0, 0
			avm1, avm2, avm3 = 0, 0, 0
			avtmp = 0
			n, nm = 0, 0
			t0, t0m = t, tm
		case <-mpu.cClose: // Stop the goroutine, ease up on the CPU
			break
		}
	}
}

// CloseMPU stops the driver from reading the MPU.
//TODO westphae: need a way to start it going again!
func (mpu *BMX160) CloseMPU() {
	mpu.cClose <- true
}

// SetSampleRate changes the sampling rate of the MPU.
//we use the OSR4 filter, which results in a bandwidth four times 
//lower than the 3dB cutoff frequencies listed in the data sheet
func (mpu *BMX160) SetSampleRateGyro(rate int) (err error) {

	var odr byte
	switch {
	case rate >= 3200:
		odr = BMI160_GYRO_ODR_3200HZ
	case rate >= 1600:
		odr = BMI160_GYRO_ODR_1600HZ
	case rate >= 800:
		odr = BMI160_GYRO_ODR_800HZ
	case rate >= 400:
		odr = BMI160_GYRO_ODR_400HZ
	case rate >= 200:
		odr = BMI160_GYRO_ODR_200HZ
	case rate >= 100:
		odr = BMI160_GYRO_ODR_100HZ
	case rate >= 50:
		odr = BMI160_GYRO_ODR_50HZ
	default:
		odr = BMI160_GYRO_ODR_25HZ
	}

	errWrite := mpu.i2cWrite(BMI160_GYRO_CONFIG_ADDR, odr)
	if errWrite != nil {
		err = fmt.Errorf("BMX160 Error: couldn't set Gyro Sample Rate: %s", errWrite.Error())
	}
        
	value, errWrite := mpu.i2cRead(BMI160_ERROR_REG_ADDR)
        if value != 0 {
	   err = fmt.Errorf("Error setting sampling rate. Value not allowed read the data sheet on gyr_conf: %x", value)
        }
	return
}

// SetSampleRate changes the sampling rate of the MPU.
//we use the OSR4 filter, which results in a bandwidth four times 
//lower than the 3dB cutoff frequencies listed in the data sheet
func (mpu *BMX160) SetSampleRateAccel(rate int) (err error) {

	var odr byte
	switch {
	case rate >= 1600:
		odr = BMI160_ACCEL_ODR_1600HZ
	case rate >= 800:
		odr =  BMI160_ACCEL_ODR_800HZ
	case rate >= 400:
		odr =  BMI160_ACCEL_ODR_400HZ
	case rate >= 200:
		odr =  BMI160_ACCEL_ODR_200HZ
	case rate >= 100:
		odr =  BMI160_ACCEL_ODR_100HZ
	case rate >= 50:
		odr =  BMI160_ACCEL_ODR_50HZ
	case rate >= 25:
		odr =  BMI160_ACCEL_ODR_25HZ
	default:
		odr = BMI160_ACCEL_ODR_12_5HZ
	}

	errWrite := mpu.i2cWrite(BMI160_ACCEL_CONFIG_ADDR, odr)
	if errWrite != nil {
		err = fmt.Errorf("BMX160 Error: couldn't set accelerometer Sample Rate: %s", errWrite.Error())
	}
        
	value, errWrite := mpu.i2cRead(BMI160_ERROR_REG_ADDR)
        if value != 0 {
	   err = fmt.Errorf("Error setting sampling rate for accelerometer. Value not allowed read the data sheet on gyr_conf: %x", value)
        }
	return
}

//GetChip ID
func (mpu *BMX160) GetChipID() (id byte, err error) {
	id, errRead := mpu.i2cRead(BMI160_CHIP_ID_ADDR)
	if errRead != nil {
		return 0, errors.New("Error reading BMX160 chip ID")
	}

	return
}


// SampleRate returns the current sample rate of the BMX160, in Hz.
func (mpu *BMX160) SampleRate() int {
	return mpu.sampleRate
}

// MagEnabled returns whether or not the magnetometer is being read.
func (mpu *BMX160) MagEnabled() bool {
	return mpu.enableMag
}

// SetGyroSensitivity sets the gyro sensitivity of the MPU9250; it must be one of the following values:
// 250, 500, 1000, 2000 (all in °/s).
func (mpu *BMX160) SetGyroSensitivity(sensitivityGyro int) (err error) {
	var sensGyro byte

	switch sensitivityGyro {
	case 2000:
		sensGyro = BMI160_GYRO_RANGE_2000_DPS
		mpu.scaleGyro = 2000.0 / float64(math.MaxInt16)
	case 1000:
		sensGyro = BMI160_GYRO_RANGE_1000_DPS 
		mpu.scaleGyro = 1000.0 / float64(math.MaxInt16)
	case 500:
		sensGyro = BMI160_GYRO_RANGE_500_DPS 
		mpu.scaleGyro = 500.0 / float64(math.MaxInt16)
	case 250:
		sensGyro = BMI160_GYRO_RANGE_250_DPS 
		mpu.scaleGyro = 250.0 / float64(math.MaxInt16)
	case 125:
		sensGyro = BMI160_GYRO_RANGE_125_DPS 
		mpu.scaleGyro = 125.0 / float64(math.MaxInt16)
	default:
		err = fmt.Errorf("BMX160 Error: %d is not a valid gyro sensitivity", sensitivityGyro)
	}

	if errWrite := mpu.i2cWrite(BMI160_GYRO_RANGE_ADDR, sensGyro); errWrite != nil {
		err = errors.New("BMX160: couldn't set gyro sensitivity")
	}

	return
}

// SetAccelSensitivity sets the accelerometer sensitivity of the BMX160; it must be one of the following values:
// 2, 4, 8, 16, all in G (gravity).
func (mpu *BMX160) SetAccelSensitivity(sensitivityAccel int) (err error) {
	var sensAccel byte

	switch sensitivityAccel {
	case 16:
		sensAccel = BMI160_ACCEL_RANGE_16G
		mpu.scaleAccel = 16.0 / float64(math.MaxInt16)
	case 8:
		sensAccel = BMI160_ACCEL_RANGE_8G
		mpu.scaleAccel = 8.0 / float64(math.MaxInt16)
	case 4:
		sensAccel = BMI160_ACCEL_RANGE_4G
		mpu.scaleAccel = 4.0 / float64(math.MaxInt16)
	case 2:
		sensAccel = BMI160_ACCEL_RANGE_2G
		mpu.scaleAccel = 2.0 / float64(math.MaxInt16)
	default:
		err = fmt.Errorf("BMX160 Error: %d is not a valid accel sensitivity", sensitivityAccel)
	}

	if errWrite := mpu.i2cWrite(BMI160_ACCEL_RANGE_ADDR, sensAccel); errWrite != nil {
		err = errors.New("BMX160 Error: couldn't set accel sensitivity")
	}

	return
}

// Fast Offset Compensation of gyroscope and accelerometer
func (mpu *BMX160) FastOffsetCompensation() error {


	if err := mpu.i2cWrite(BMI160_FOC_CONF_ADDR,BMI160_GYRO_FOC_EN_MSK|BMI160_ACCEL_FOC_X_CONF_MSK|BMI160_ACCEL_FOC_Y_CONF_MSK|BMI160_ACCEL_FOC_Z_CONF_MSK); err != nil {
		return errors.New("BMX160 Error: couldn't configure fast offset calibration")
	}

        //trigger FOC
        if err := mpu.i2cWrite(BMI160_COMMAND_REG_ADDR,BMI160_START_FOC_CMD); err != nil { 
               return errors.New("Error start fast offset compensation BMX160")
        }
        time.Sleep(500 * time.Millisecond) // wait time for FOC to finish

	stat, err := mpu.i2cRead(BMI160_STATUS_ADDR)
	if err != nil {
		return errors.New("BMX160 Error: Reading Status Byte after FOC")
	}
        if stat&BMI160_FOC_STATUS_MSK != 0x8 {
		return errors.New("BMX160 Error: Couldn't complete FOC")
        }

	return nil
}

// Enable Offset Compensation for gyroscope and accelerometer
func (mpu *BMX160) EnableOffsetCompensation() error {

       //Get the OFFSET byte that also holds the 9:8 bits of the gyro offsets, we don't want to disturb those bits
       stat, err := mpu.i2cRead(BMI160_OFFSET_CONF_ADDR)
	if err != nil {
		return errors.New("BMX160 Error: Reading Offset config byte")
	}

       //if err := mpu.i2cWrite(BMI160_OFFSET_CONF_ADDR,stat|BMI160_GYRO_OFFSET_EN_MSK|BMI160_ACCEL_OFFSET_EN_MSK); err != nil {
       if err := mpu.i2cWrite(BMI160_OFFSET_CONF_ADDR,stat|BMI160_GYRO_OFFSET_EN_MSK); err != nil {
		return errors.New("BMX160 Error: couldn't enable to offset")
	}

	stat, err = mpu.i2cRead(BMI160_OFFSET_CONF_ADDR)
	if err != nil {
		return errors.New("BMX160 Error: Reading Offset config byte")
	}


        //if stat&(BMI160_GYRO_OFFSET_EN_MSK|BMI160_ACCEL_OFFSET_EN_MSK) != 0xC0 {
	//	return errors.New("BMX160 Error: Couldn't enable offset compensation")
        //}

	return nil
}


func (mpu *BMX160) i2cWrite(register, value byte) (err error) {

	if errWrite := mpu.i2cbus.WriteByteToReg(BMI160_I2C_ADDR, register, value); errWrite != nil {
		err = fmt.Errorf("BMX160 Error writing %X to %X: %s\n",
			value, register, errWrite.Error())
	} else {
		time.Sleep(time.Millisecond)
	}
	return
}

func (mpu *BMX160) i2cReadBytes(register byte, value []byte) (err error) {
	errRead := (mpu.i2cbus).ReadFromReg(BMI160_I2C_ADDR, register, value)
	if errRead != nil {
		err = fmt.Errorf("BMX160 Error reading from %X: %s", register, err)
	}
	return
}

func (mpu *BMX160) i2cRead(register byte) (value uint8, err error) {
	value, errWrite := mpu.i2cbus.ReadByteFromReg(BMI160_I2C_ADDR, register)
	if errWrite != nil {
		err = fmt.Errorf("i2cRead error: %s", errWrite.Error())
	}
	return
}

func (mpu *BMX160) i2cRead2(register byte) (value int16, err error) {

        buf := make([]byte, 2)
	if err := mpu.i2cbus.ReadFromReg(BMI160_I2C_ADDR, register, buf); err != nil {
		err = fmt.Errorf("BMX160 Error reading %x: %s\n", register, err.Error())
		return 0, err
	}

	value =  int16((uint16(buf[1]) << 8) | uint16(buf[0]))

	return
}


/*!
 * @brief This internal API reads the trim registers of the sensor and stores
 * the trim values in the "trim_data" of device structure.
 * Taken from Boschs BMM150 API
 */
func (mpu *BMX160)  read_trim_registers() (err error) {
 var (
       trim_x1y1 = make([]byte, 2) 
       trim_xyz_data = make([]byte,4)
       trim_xy1xy2 = make([]byte,10)
       temp_msb uint16
     )

    /* Trim register value is read */
    if err:= mpu.bmm150_get_regs(BMM150_DIG_X1, trim_x1y1); err == nil {
        if err = mpu.bmm150_get_regs(BMM150_DIG_Z4_LSB, trim_xyz_data); err == nil {
            if err = mpu.bmm150_get_regs(BMM150_DIG_Z2_LSB, trim_xy1xy2); err == nil {
                /* Trim data which is read is updated
                 * in the device structure */
                mpu.magnSensTrimValues.dig_x1 = trim_x1y1[0]
                mpu.magnSensTrimValues.dig_y1 = trim_x1y1[1]
                mpu.magnSensTrimValues.dig_x2 = trim_xyz_data[2]
                mpu.magnSensTrimValues.dig_y2 = trim_xyz_data[3]
                temp_msb = ((uint16)(trim_xy1xy2[3])) << 8
                mpu.magnSensTrimValues.dig_z1 = (uint16)(temp_msb | (uint16)(trim_xy1xy2[2]))
                temp_msb = ((uint16)(trim_xy1xy2[1])) << 8
                mpu.magnSensTrimValues.dig_z2 = (int16)(temp_msb | (uint16)(trim_xy1xy2[0]))
                temp_msb = ((uint16)(trim_xy1xy2[7])) << 8
                mpu.magnSensTrimValues.dig_z3 = (int16)(temp_msb | (uint16)(trim_xy1xy2[6]))
                temp_msb = ((uint16)(trim_xyz_data[1])) << 8
                mpu.magnSensTrimValues.dig_z4 = (int16)(temp_msb | (uint16)(trim_xyz_data[0]))
                mpu.magnSensTrimValues.dig_xy1 = trim_xy1xy2[9]
                mpu.magnSensTrimValues.dig_xy2 = trim_xy1xy2[8]
                temp_msb = ((uint16)(trim_xy1xy2[5] & 0x7F)) << 8
                mpu.magnSensTrimValues.dig_xyz1 = (uint16)(temp_msb | (uint16)(trim_xy1xy2[4]))
            }
        }
    }
    
    

    return err;
}

func (mpu *BMX160)  bmm150_get_regs(register byte, data []byte) (err error) {

       //entering setup mode MAG_IF[0]<7> = 1 
//      if err := mpu.i2cWrite(BMI160_AUX_IF_1_ADDR, BMI160_MANUAL_MODE_EN_MSK); err != nil {
 //         return errors.New("BMX160 Error: couldn't disable interrupts")
 //      }
 //      time.Sleep(time.Millisecond) // wait for command to trickle through 

       for i := 0; i < len(data); i++ {
          //fmt.Printf("get bmm150 address %x\n", register+(byte)(i))
          if err = mpu.i2cWrite(BMI160_AUX_IF_2_ADDR,register+(byte)(i)); err != nil {
   	    return errors.New("BMX160 Error: couldn't set BMM150 address to read")
          }
        time.Sleep(BMI160_AUX_COM_DELAY*time.Millisecond) // wait for command to trickle through

	//stat, err := mpu.i2cRead(BMI160_STATUS_ADDR)
        //fmt.Printf("The status bit (should be 0 if complete): %x\n",stat&0x04)

          var tmp byte
          if tmp, err = mpu.i2cbus.ReadByteFromReg(BMI160_I2C_ADDR,BMI160_AUX_DATA_ADDR); err != nil {
		err = fmt.Errorf("BMX160 Error reading %x: %s\n", register, err.Error())
		return err
	  }  
          fmt.Printf("%x %x\n",register+(byte)(i), tmp)
          data[i] = tmp
        }
      
       return nil
}

func (mpu *BMX160)  bmm150_set_regs(register byte, data byte) (err error) {

//      if err := mpu.i2cWrite(BMI160_AUX_IF_1_ADDR, BMI160_MANUAL_MODE_EN_MSK); err != nil {
//          return errors.New("BMX160 Error: couldn't disable interrupts")
 //      }
 //      time.Sleep(time.Millisecond) // wait for command to trickle through 

    if err := mpu.i2cWrite(BMI160_AUX_IF_4_ADDR, register); err != nil {
             return errors.New("BMX160 Error setting bmm150 register to write to")
    }
    time.Sleep(10*BMI160_AUX_COM_DELAY*time.Millisecond) // wait for command to trickle through 
    if err := mpu.i2cWrite(BMI160_AUX_IF_3_ADDR, data); err != nil {
              return errors.New("BMX160 Error setting value to write into bmm150")
    }
    time.Sleep(10*BMI160_AUX_COM_DELAY*time.Millisecond) // wait for command to trickle through 

	stat, err := mpu.i2cRead(BMI160_STATUS_ADDR)
        fmt.Printf("writing status %x\n",stat&0x04)

      
       return nil
}

