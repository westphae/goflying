package ahrsweb

import (
	"encoding/json"
	"log"
	"time"
	//"math"
	"net/url"

	"..//ahrs"
	"fmt"
	"github.com/gorilla/websocket"
)

type KalmanListener struct {
	data *AHRSData
	c    *websocket.Conn
}

func NewKalmanListener() (kl *KalmanListener, err error) {
	kl = new(KalmanListener)
	kl.data = new(AHRSData)
	if err = kl.connect(); err != nil {
		return nil, err
	}

	return kl, nil
}

func (kl *KalmanListener) connect() (err error) {
	u := url.URL{Scheme: "ws", Host: fmt.Sprintf("localhost:%d", Port), Path: "/ahrsweb"}
	kl.c, _, err = websocket.DefaultDialer.Dial(u.String(), nil)
	return
}

func (kl *KalmanListener) update(s *ahrs.State, m *ahrs.Measurement) {
	kl.data.T = float64(time.Now().UnixNano()/1000) / 1e6

	if s != nil {
		kl.data.U1 = s.U1
		kl.data.U2 = s.U2
		kl.data.U3 = s.U3
		kl.data.Z1 = s.Z1
		kl.data.Z2 = s.Z2
		kl.data.Z3 = s.Z3
		kl.data.E0 = s.E0
		kl.data.E1 = s.E1
		kl.data.E2 = s.E2
		kl.data.E3 = s.E3
		kl.data.H1 = s.H1
		kl.data.H2 = s.H2
		kl.data.H3 = s.H3
		kl.data.N1 = s.N1
		kl.data.N2 = s.N2
		kl.data.N3 = s.N3

		if s.M != nil {
			kl.data.DU1 = s.M.Get(0, 0)
			kl.data.DU2 = s.M.Get(1, 1)
			kl.data.DU3 = s.M.Get(2, 2)
			kl.data.DZ1 = s.M.Get(3, 3)
			kl.data.DZ2 = s.M.Get(4, 4)
			kl.data.DZ3 = s.M.Get(5, 5)
			kl.data.DE0 = s.M.Get(6, 6)
			kl.data.DE1 = s.M.Get(7, 7)
			kl.data.DE2 = s.M.Get(8, 8)
			kl.data.DE3 = s.M.Get(9, 9)
			kl.data.DH1 = s.M.Get(10, 10)
			kl.data.DH2 = s.M.Get(11, 11)
			kl.data.DH3 = s.M.Get(12, 12)
			kl.data.DN1 = s.M.Get(13, 13)
			kl.data.DN2 = s.M.Get(14, 14)
			kl.data.DN3 = s.M.Get(15, 15)

			kl.data.DV1 = s.M.Get(16, 16)
			kl.data.DV2 = s.M.Get(17, 17)
			kl.data.DV3 = s.M.Get(18, 18)
			kl.data.DC1 = s.M.Get(19, 19)
			kl.data.DC2 = s.M.Get(20, 20)
			kl.data.DC3 = s.M.Get(21, 21)
			kl.data.DF0 = s.M.Get(22, 22)
			kl.data.DF1 = s.M.Get(23, 23)
			kl.data.DF2 = s.M.Get(24, 24)
			kl.data.DF3 = s.M.Get(25, 25)
			kl.data.DD1 = s.M.Get(26, 26)
			kl.data.DD2 = s.M.Get(27, 27)
			kl.data.DD3 = s.M.Get(28, 28)
			kl.data.DL1 = s.M.Get(29, 29)
			kl.data.DL2 = s.M.Get(30, 30)
			kl.data.DL3 = s.M.Get(31, 31)
		}

		kl.data.V1 = s.V1
		kl.data.V2 = s.V2
		kl.data.V3 = s.V3
		kl.data.C1 = s.C1
		kl.data.C2 = s.C2
		kl.data.C3 = s.C3
		kl.data.F0 = s.F0
		kl.data.F1 = s.F1
		kl.data.F2 = s.F2
		kl.data.F3 = s.F3
		kl.data.D1 = s.D1
		kl.data.D2 = s.D2
		kl.data.D3 = s.D3
		kl.data.L1 = s.L1
		kl.data.L2 = s.L2
		kl.data.L3 = s.L3

		roll, pitch, heading := ahrs.FromQuaternion(s.E0, s.E1, s.E2, s.E3)
		kl.data.Pitch = pitch / ahrs.Deg
		kl.data.Roll = roll / ahrs.Deg
		kl.data.Heading = heading / ahrs.Deg
	} else {
		log.Println("AHRSWeb: state is nil, not updating data")
	}

	if m != nil {
		kl.data.UValid = m.UValid
		kl.data.WValid = m.WValid
		kl.data.SValid = m.SValid
		kl.data.MValid = m.MValid

		kl.data.S1 = m.U1
		kl.data.S2 = m.U2
		kl.data.S3 = m.U3
		kl.data.W1 = m.W1
		kl.data.W2 = m.W2
		kl.data.W3 = m.W3
		kl.data.A1 = m.A1
		kl.data.A2 = m.A2
		kl.data.A3 = m.A3
		kl.data.B1 = m.B1
		kl.data.B2 = m.B2
		kl.data.B3 = m.B3
		kl.data.M1 = m.M1
		kl.data.M2 = m.M2
		kl.data.M3 = m.M3
	} else {
		log.Println("AHRSWeb: measurement is nil, not updating data")
	}
}

func (kl *KalmanListener) Send(s *ahrs.State, m *ahrs.Measurement) error {
	kl.update(s, m)

	if msg, err := json.Marshal(kl.data); err != nil {
		log.Println("AHRSWeb: Error marshalling json data:", err)
		log.Println("AHRSWeb: Data was:", kl.data)
		return err
	} else {
		if err := kl.c.WriteMessage(websocket.TextMessage, msg); err != nil {
			log.Println("AHRSWeb: Error writing to websocket:", err)
			err2 := kl.connect()
			return fmt.Errorf("AHRSWeb: %v: %v", err, err2) // Just drop this message
		}
	}
	return nil
}

func (kl *KalmanListener) Close() {
	if err := kl.c.WriteMessage(websocket.CloseMessage, websocket.FormatCloseMessage(websocket.CloseNormalClosure, "")); err != nil {
		log.Println("AHRSWeb: Error closing websocket:", err)
		return
	}
	kl.c.Close()
}
