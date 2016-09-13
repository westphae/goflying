package main

import (
	"encoding/json"
	"log"
	"math/rand"
	"time"
	"math"
)

type RandListener struct {
	r    *room
	data *AHRSData
}

func (ml *RandListener) SetRoom(r *room) {
	ml.r = r
}

func (ml *RandListener) Init() {
	ml.data = new(AHRSData)
}

func (ml *RandListener) Close() {
}

func (ml *RandListener) GetData() *AHRSData {
	return ml.data
}

func (ml *RandListener) update() {
	log.Println("Making up some AHRS data")

	ml.data.T = float64(time.Now().UnixNano()/1000)/1e6

	ml.data.U1 = 0.9*ml.data.U1 + 0.1*(80*rand.Float64())
	ml.data.U2 = 0.9*ml.data.U2 + 0.1*(80*rand.Float64())
	ml.data.U3 = 0.9*ml.data.U3 + 0.1*(10*rand.Float64())
	ml.data.Z1 = 0.9*ml.data.Z1 + 0.1*(0.2*rand.Float64())
	ml.data.Z2 = 0.9*ml.data.Z2 + 0.1*(0.2*rand.Float64())
	ml.data.Z3 = 0.9*ml.data.Z3 + 0.1*(0.2*rand.Float64())
	ml.data.E0 = 0.9*ml.data.E0 + 0.1*(0.2*rand.Float64())
	ml.data.E1 = 0.9*ml.data.E1 + 0.1*(0.2*rand.Float64())
	ml.data.E2 = 0.9*ml.data.E2 + 0.1*(0.2*rand.Float64())
	ml.data.E3 = 0.9*ml.data.E3 + 0.1*(0.2*rand.Float64())
	n := math.Sqrt(ml.data.E0*ml.data.E0 + ml.data.E1*ml.data.E1 + ml.data.E2*ml.data.E2 + ml.data.E3*ml.data.E3)
	ml.data.E0 /= n
	ml.data.E1 /= n
	ml.data.E2 /= n
	ml.data.E3 /= n
	ml.data.H1 = 0.9*ml.data.H1 + 0.1*(rand.Float64())
	ml.data.H2 = 0.9*ml.data.H2 + 0.1*(rand.Float64())
	ml.data.H3 = 0.9*ml.data.H3 + 0.1*(rand.Float64())
	ml.data.N1 = 0.9*ml.data.N1 + 0.1*(500*rand.Float64())
	ml.data.N2 = 0.9*ml.data.N2 + 0.1*(500*rand.Float64())
	ml.data.N3 = 0.9*ml.data.N3 + 0.1*(500*rand.Float64())

	ml.data.DU1 = 20
	ml.data.DU2 = 20
	ml.data.DU3 = 2
	ml.data.DZ1 = 0.1
	ml.data.DZ2 = 0.1
	ml.data.DZ3 = 0.1
	ml.data.DE0 = 0.1
	ml.data.DE1 = 0.1
	ml.data.DE2 = 0.1
	ml.data.DE3 = 0.1
	ml.data.DH1 = 0.5
	ml.data.DH2 = 0.5
	ml.data.DH3 = 0.5
	ml.data.DN1 = 50
	ml.data.DN2 = 50
	ml.data.DN3 = 50

	ml.data.DV1 = 2
	ml.data.DV2 = 2
	ml.data.DV3 = 0.2
	ml.data.DC1 = 0.01
	ml.data.DC2 = 0.01
	ml.data.DC3 = 0.01
	ml.data.DF0 = 0.01
	ml.data.DF1 = 0.01
	ml.data.DF2 = 0.01
	ml.data.DF3 = 0.01
	ml.data.DD1 = 0.05
	ml.data.DD2 = 0.05
	ml.data.DD3 = 0.05
	ml.data.DL1 = 5
	ml.data.DL2 = 5
	ml.data.DL3 = 5

	ml.data.V1 = 0.99*ml.data.V1 + 0.01*(20*rand.Float64())
	ml.data.V2 = 0.99*ml.data.V2 + 0.01*(20*rand.Float64())
	ml.data.V3 = 0.99*ml.data.V3 + 0.01*(20*rand.Float64())
	ml.data.C1 = 0.99*ml.data.C1 + 0.01*(0.02*rand.Float64())
	ml.data.C2 = 0.99*ml.data.C2 + 0.01*(0.02*rand.Float64())
	ml.data.C3 = 0.99*ml.data.C3 + 0.01*(0.02*rand.Float64())
	ml.data.F0 = 0.99*ml.data.F0 + 0.01*(0.02*rand.Float64())
	ml.data.F1 = 0.99*ml.data.F1 + 0.01*(0.02*rand.Float64())
	ml.data.F2 = 0.99*ml.data.F2 + 0.01*(0.02*rand.Float64())
	ml.data.F3 = 0.99*ml.data.F3 + 0.01*(0.02*rand.Float64())
	n = math.Sqrt(ml.data.F0*ml.data.F0 + ml.data.F1*ml.data.F1 + ml.data.F2*ml.data.F2 + ml.data.F3*ml.data.F3)
	ml.data.F0 /= n
	ml.data.F1 /= n
	ml.data.F2 /= n
	ml.data.F3 /= n
	ml.data.D1 = 0.99*ml.data.D1 + 0.01*(0.01*rand.Float64())
	ml.data.D2 = 0.99*ml.data.D2 + 0.01*(0.01*rand.Float64())
	ml.data.D3 = 0.99*ml.data.D3 + 0.01*(0.01*rand.Float64())
	ml.data.L1 = 0.99*ml.data.L1 + 0.01*(50*rand.Float64())
	ml.data.L2 = 0.99*ml.data.L2 + 0.01*(50*rand.Float64())
	ml.data.L3 = 0.99*ml.data.L3 + 0.01*(50*rand.Float64())

	ml.data.Pitch = 20*math.Sin(ml.data.T/60*math.Pi)
	ml.data.Roll = 55*math.Sin(ml.data.T/60*math.Pi)
	ml.data.Heading = math.Mod(ml.data.T/60*720, 360)

	if m := rand.Intn(100); m<90 {
		ml.data.UValid = !ml.data.UValid
	}

	if m := rand.Intn(100); m<90 {
		ml.data.WValid = !ml.data.WValid
	}

	if m := rand.Intn(100); m<90 {
		ml.data.SValid = !ml.data.SValid
	}

	if m := rand.Intn(100); m<90 {
		ml.data.MValid = !ml.data.MValid
	}

	ml.data.S1 = 0
	ml.data.S2 = 0
	ml.data.S3 = 0
	ml.data.W1 = 0.9*ml.data.W1 + 0.1*(50*rand.Float64())
	ml.data.W2 = 0.9*ml.data.W2 + 0.1*(50*rand.Float64())
	ml.data.W3 = 0.9*ml.data.W3 + 0.1*(50*rand.Float64())
	ml.data.A1 = ml.data.Z1 + ml.data.C1 + 0.05*rand.Float64()
	ml.data.A2 = ml.data.Z2 + ml.data.C2 + 0.05*rand.Float64()
	ml.data.A3 = 1 + ml.data.Z3 + ml.data.C3 + 0.05*rand.Float64()
	ml.data.B1 = ml.data.H1 + ml.data.D1 + 0.1*rand.Float64()
	ml.data.B2 = ml.data.H2 + ml.data.D2 + 0.1*rand.Float64()
	ml.data.B3 = ml.data.H3 + ml.data.D3 + 0.1*rand.Float64()
	ml.data.M1 = ml.data.N1 + ml.data.L1 + 10*rand.Float64()
	ml.data.M2 = ml.data.N2 + ml.data.L2 + 10*rand.Float64()
	ml.data.M3 = ml.data.N3 + ml.data.L3 + 10*rand.Float64()
}

func (ml *RandListener) Run() {
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
