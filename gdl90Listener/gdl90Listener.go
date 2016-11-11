package main

import (
	"errors"
	"flag"
	"fmt"
	"log"
	"net"
)

// iLevil FLAG, CompanyID, PackageID and Version
const (
	FLAG     = 0x7e  // iLevil FLAG symbol
	CID      = "LE"  // iLevil CompanyID
	PIDAHRS  = 0x01  // AHRS PackageID
	PIDAHRS1 = 0x01  // AHRS PackageID Version 1
	INTERR   = 32767 // Error value for int16 types
	UINTERR  = 65535 // Error value for uint16 types
)

var ahrsErrorStr = "AHRS Message Error: %s"

// AHRSMsg contains the data from an iLevil AHRS version 1 message
type AHRSMsg struct {
	companyID        string
	packageID        byte
	packageIDVersion byte
	roll             int16
	pitch            int16
	yaw              int16
	inclination      int16
	turnCoord        int16
	gLoad            int16
	kias             int16
	pAlt             uint16
	vertSpeed        int16
}

func (dat *AHRSMsg) decodeAHRSMsg(msg []byte) (err error) {
	if len(msg) != 28 {
		err = fmt.Errorf(ahrsErrorStr, fmt.Sprintf("Message length was %d, should be 28", len(msg)))
		return
	}
	if msg[0] != FLAG {
		err = fmt.Errorf(ahrsErrorStr, "Missing FLAG 0x7E at message beginning")
		return
	}
	if msg[27] != FLAG {
		err = fmt.Errorf(ahrsErrorStr, "Missing FLAG 0x7E at message end")
		return
	}
	if s := string(msg[1:3]); s != CID {
		err = fmt.Errorf(ahrsErrorStr, fmt.Sprintf("Incorrect CompanyID: %s", string(msg[1:3])))
		return
	} else {
		dat.companyID = s
	}
	if s := msg[3]; s != PIDAHRS {
		err = fmt.Errorf(ahrsErrorStr, fmt.Sprintf("Expecting ProductID 0x01, received %b", msg[3]))
		return
	} else {
		dat.packageID = s
	}
	if s := msg[4]; s != PIDAHRS1 {
		err = fmt.Errorf(ahrsErrorStr, fmt.Sprintf("Expecting ProductID Version 0x01, received %b", msg[4]))
		return
	} else {
		dat.packageIDVersion = s
	}
	if !crcCheck(msg) {
		err = fmt.Errorf(ahrsErrorStr, "CRC incorrect")
		return
	}

	dat.roll = bytes2int(msg[5:7])
	dat.pitch = bytes2int(msg[7:9])
	dat.yaw = bytes2int(msg[9:11])
	dat.inclination = bytes2int(msg[11:13])
	dat.turnCoord = bytes2int(msg[13:15])
	dat.gLoad = bytes2int(msg[15:17])
	dat.kias = bytes2int(msg[17:19])
	dat.pAlt = bytes2uint(msg[19:21])
	dat.vertSpeed = bytes2int(msg[21:23])

	return
}

func (dat *AHRSMsg) Roll() (roll float64, err error) {
	if dat.roll == INTERR {
		err = errors.New("Bad Roll value")
	} else {
		roll = float64(dat.roll) / 10
	}
	return
}

func (dat *AHRSMsg) Pitch() (pitch float64, err error) {
	if dat.pitch == INTERR {
		err = errors.New("Bad Pitch value")
	} else {
		pitch = float64(dat.pitch) / 10
	}
	return
}

func (dat *AHRSMsg) Yaw() (yaw float64, err error) {
	if dat.yaw == INTERR {
		err = errors.New("Bad Yaw value")
	} else {
		yaw = float64(dat.yaw) / 10
	}
	return
}

func (dat *AHRSMsg) Inclination() (inclination float64, err error) {
	if dat.inclination == INTERR {
		err = errors.New("Bad Inclination value")
	} else {
		inclination = float64(dat.inclination) / 10
	}
	return
}

func (dat *AHRSMsg) TurnCoord() (turnCoord float64, err error) {
	if dat.turnCoord == INTERR {
		err = errors.New("Bad TurnCoord value")
	} else {
		turnCoord = float64(dat.turnCoord) / 10
	}
	return
}

func (dat *AHRSMsg) GLoad() (gLoad float64, err error) {
	if dat.gLoad == INTERR {
		err = errors.New("Bad GLoad value")
	} else {
		gLoad = float64(dat.gLoad) / 10
	}
	return
}

func (dat *AHRSMsg) KIAS() (kias float64, err error) {
	if dat.kias == INTERR {
		err = errors.New("Bad KIAS value")
	} else {
		kias = float64(dat.kias) / 10
	}
	return
}

func (dat *AHRSMsg) PAlt() (pAlt float64, err error) {
	if dat.pAlt == UINTERR {
		err = errors.New("Bad PAlt value")
	} else {
		pAlt = float64(dat.pAlt) - 5000
	}
	return
}

func (dat *AHRSMsg) VertSpeed() (vertSpeed float64, err error) {
	if dat.vertSpeed == INTERR {
		err = errors.New("Bad VertSpeed value")
	} else {
		vertSpeed = float64(dat.vertSpeed)
	}
	return
}

func bytes2int(b []byte) int16 {
	return (int16(b[1]) << 0) | (int16(b[0]) << 8)
}

func bytes2uint(b []byte) uint16 {
	return (uint16(b[1]) << 0) | (uint16(b[0]) << 8)
}

func crcCheck(msg []byte) bool {
	return true //TODO westphae: implement this
}

func main() {
	var (
		ipAddress   string
		n           int
		err         error
		roll        float64
		pitch       float64
		yaw         float64
		inclination float64
		turnCoord   float64
		gLoad       float64
		kias        float64
		pAlt        float64
		vertSpeed   float64
	)

	if len(flag.Args()) == 0 {
		ipAddress = ""
	} else {
		ipAddress = flag.Args()[0]
	}

	conn, err := net.ListenPacket("udp", ipAddress+":4000")
	if err != nil {
		log.Fatalf("Couldn't dial UDP: %v\n", err)
	}
	defer conn.Close()

	ahrsMsg := new(AHRSMsg)
	buffer := make([]byte, 1024)
	for {
		n, _, err = conn.ReadFrom(buffer)
		if err != nil {
			log.Printf("Error: %v\n", err)
		} else {
			// log.Printf("Received %s (len %d) from %s\n", string(buffer[0:n]), n, addr)
			err = ahrsMsg.decodeAHRSMsg(buffer[0:n])
			if err != nil {
				continue
				// log.Println(err)
			} else {
				roll, err = ahrsMsg.Roll()
				if err == nil {
					log.Printf("%12s %+3.1f", "Roll", roll)
				}
				pitch, err = ahrsMsg.Pitch()
				if err == nil {
					log.Printf("%12s  %+2.1f", "Pitch", pitch)
				}
				yaw, err = ahrsMsg.Yaw()
				if err == nil {
					log.Printf("%12s %+3.1f", "Yaw", yaw)
				}
				inclination, err = ahrsMsg.Inclination()
				if err == nil {
					log.Printf("%12s  %+2.1f", "Inclination", inclination)
				}
				turnCoord, err = ahrsMsg.TurnCoord()
				if err == nil {
					log.Printf("%12s %+3.1f", "TurnCoord", turnCoord)
				}
				gLoad, err = ahrsMsg.GLoad()
				if err == nil {
					log.Printf("%12s   %+1.1f", "GLoad", gLoad)
				}
				kias, err = ahrsMsg.KIAS()
				if err == nil {
					log.Printf("%12s %+4.1f", "KIAS", kias)
				}
				pAlt, err = ahrsMsg.PAlt()
				if err == nil {
					log.Printf("%12s %+5.0f", "PAlt", pAlt)
				}
				vertSpeed, err = ahrsMsg.VertSpeed()
				if err == nil {
					log.Printf("%12s %+5.0f", "VertSpeed", vertSpeed)
				}
				log.Println()
			}
		}
	}
}
