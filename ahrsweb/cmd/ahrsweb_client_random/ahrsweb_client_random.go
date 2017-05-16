/*
Client adapted from echo example in github.com/gorilla/websocket/examples/echo
*/

package main

import (
	"encoding/json"
	"flag"
	"log"
	"math"
	"math/rand"
	"os"
	"time"

	"../../ahrsweb"
	"fmt"
	"github.com/gorilla/websocket"
	"net/url"
	"os/signal"
)

func update(data *ahrsweb.AHRSData) {
	log.Println("Making up some AHRS data")

	data.T = float64(time.Now().UnixNano()/1000) / 1e6

	data.U1 = 0.9*data.U1 + 0.1*(80*rand.Float64())
	data.U2 = 0.9*data.U2 + 0.1*(80*rand.Float64())
	data.U3 = 0.9*data.U3 + 0.1*(10*rand.Float64())
	data.Z1 = 0.9*data.Z1 + 0.1*(0.2*rand.Float64())
	data.Z2 = 0.9*data.Z2 + 0.1*(0.2*rand.Float64())
	data.Z3 = 0.9*data.Z3 + 0.1*(0.2*rand.Float64())
	data.E0 = 0.9*data.E0 + 0.1*(0.2*rand.Float64())
	data.E1 = 0.9*data.E1 + 0.1*(0.2*rand.Float64())
	data.E2 = 0.9*data.E2 + 0.1*(0.2*rand.Float64())
	data.E3 = 0.9*data.E3 + 0.1*(0.2*rand.Float64())
	n := math.Sqrt(data.E0*data.E0 + data.E1*data.E1 + data.E2*data.E2 + data.E3*data.E3)
	data.E0 /= n
	data.E1 /= n
	data.E2 /= n
	data.E3 /= n
	data.H1 = 0.9*data.H1 + 0.1*(rand.Float64())
	data.H2 = 0.9*data.H2 + 0.1*(rand.Float64())
	data.H3 = 0.9*data.H3 + 0.1*(rand.Float64())
	data.N1 = 0.9*data.N1 + 0.1*(500*rand.Float64())
	data.N2 = 0.9*data.N2 + 0.1*(500*rand.Float64())
	data.N3 = 0.9*data.N3 + 0.1*(500*rand.Float64())

	data.DU1 = 20
	data.DU2 = 20
	data.DU3 = 2
	data.DZ1 = 0.1
	data.DZ2 = 0.1
	data.DZ3 = 0.1
	data.DE0 = 0.1
	data.DE1 = 0.1
	data.DE2 = 0.1
	data.DE3 = 0.1
	data.DH1 = 0.5
	data.DH2 = 0.5
	data.DH3 = 0.5
	data.DN1 = 50
	data.DN2 = 50
	data.DN3 = 50

	data.DV1 = 2
	data.DV2 = 2
	data.DV3 = 0.2
	data.DC1 = 0.01
	data.DC2 = 0.01
	data.DC3 = 0.01
	data.DF0 = 0.01
	data.DF1 = 0.01
	data.DF2 = 0.01
	data.DF3 = 0.01
	data.DD1 = 0.05
	data.DD2 = 0.05
	data.DD3 = 0.05
	data.DL1 = 5
	data.DL2 = 5
	data.DL3 = 5

	data.V1 = 0.99*data.V1 + 0.01*(20*rand.Float64())
	data.V2 = 0.99*data.V2 + 0.01*(20*rand.Float64())
	data.V3 = 0.99*data.V3 + 0.01*(20*rand.Float64())
	data.C1 = 0.99*data.C1 + 0.01*(0.02*rand.Float64())
	data.C2 = 0.99*data.C2 + 0.01*(0.02*rand.Float64())
	data.C3 = 0.99*data.C3 + 0.01*(0.02*rand.Float64())
	data.F0 = 0.99*data.F0 + 0.01*(0.02*rand.Float64())
	data.F1 = 0.99*data.F1 + 0.01*(0.02*rand.Float64())
	data.F2 = 0.99*data.F2 + 0.01*(0.02*rand.Float64())
	data.F3 = 0.99*data.F3 + 0.01*(0.02*rand.Float64())
	n = math.Sqrt(data.F0*data.F0 + data.F1*data.F1 + data.F2*data.F2 + data.F3*data.F3)
	data.F0 /= n
	data.F1 /= n
	data.F2 /= n
	data.F3 /= n
	data.D1 = 0.99*data.D1 + 0.01*(0.01*rand.Float64())
	data.D2 = 0.99*data.D2 + 0.01*(0.01*rand.Float64())
	data.D3 = 0.99*data.D3 + 0.01*(0.01*rand.Float64())
	data.L1 = 0.99*data.L1 + 0.01*(50*rand.Float64())
	data.L2 = 0.99*data.L2 + 0.01*(50*rand.Float64())
	data.L3 = 0.99*data.L3 + 0.01*(50*rand.Float64())

	data.Pitch = 20 * math.Sin(data.T/60*math.Pi)
	data.Roll = 55 * math.Sin(data.T/60*math.Pi)
	data.Heading = math.Mod(data.T/60*720, 360)

	if m := rand.Intn(100); m < 90 {
		data.UValid = !data.UValid
	}

	if m := rand.Intn(100); m < 90 {
		data.WValid = !data.WValid
	}

	if m := rand.Intn(100); m < 90 {
		data.SValid = !data.SValid
	}

	if m := rand.Intn(100); m < 90 {
		data.MValid = !data.MValid
	}

	data.S1 = 0
	data.S2 = 0
	data.S3 = 0
	data.W1 = 0.9*data.W1 + 0.1*(50*rand.Float64())
	data.W2 = 0.9*data.W2 + 0.1*(50*rand.Float64())
	data.W3 = 0.9*data.W3 + 0.1*(50*rand.Float64())
	data.A1 = data.Z1 + data.C1 + 0.05*rand.Float64()
	data.A2 = data.Z2 + data.C2 + 0.05*rand.Float64()
	data.A3 = 1 + data.Z3 + data.C3 + 0.05*rand.Float64()
	data.B1 = data.H1 + data.D1 + 0.1*rand.Float64()
	data.B2 = data.H2 + data.D2 + 0.1*rand.Float64()
	data.B3 = data.H3 + data.D3 + 0.1*rand.Float64()
	data.M1 = data.N1 + data.L1 + 10*rand.Float64()
	data.M2 = data.N2 + data.L2 + 10*rand.Float64()
	data.M3 = data.N3 + data.L3 + 10*rand.Float64()
}

var addr = flag.String("addr", fmt.Sprint("localhost:%d", ahrsweb.Port), "ahrsweb server address")

func main() {
	flag.Parse()

	// Catch interrupts from os so we can close everything nicely
	interrupt := make(chan os.Signal, 1)
	signal.Notify(interrupt, os.Interrupt)

	u := url.URL{Scheme: "ws", Host: *addr, Path: "/ahrsweb"}

	log.Printf("connecting to %s\n", u.String())
	c, _, err := websocket.DefaultDialer.Dial(u.String(), nil)
	if err != nil {
		log.Fatalln("dial error:", err.Error())
	}
	defer c.Close()

	data := new(ahrsweb.AHRSData)

	ticker := time.NewTicker(time.Millisecond * 100)
	defer ticker.Stop()

	for {
		select {
		case <-ticker.C:
			update(data)
			if msg, err := json.Marshal(data); err != nil {
				log.Println("Error marshalling json data: ", err.Error())
			} else {
				log.Println("Sending AHRS data to ahrsweb")
				if err := c.WriteMessage(websocket.TextMessage, msg); err != nil {
					log.Println("Error writing to websocket:", err.Error())
					return
				}
			}
		case <-interrupt:
			log.Println("Received interrupt")
			if err := c.WriteMessage(websocket.CloseMessage, websocket.FormatCloseMessage(websocket.CloseNormalClosure, "")); err != nil {
				log.Println("Error closing websocket:", err.Error())
				return
			}
			c.Close()
			return
		}
	}
}
