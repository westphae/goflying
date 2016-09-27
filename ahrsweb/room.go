package ahrsweb

import (
	"log"
	"net/http"

	"github.com/gorilla/websocket"
)

type Room struct {
	// forward is a channel that holds incoming messages
	// that should be forwarded to the other clients.
	forward chan []byte
	// join is a channel for clients wishing to join the room.
	join chan *client
	// leave is a channel for clients wishing to leave the room.
	leave chan *client
	// clients holds all current clients in this room.
	clients map[*client]bool
}

// newRoom makes a new room that is ready to go.
func NewRoom() *Room {
	return &Room{
		forward: make(chan []byte),
		join:    make(chan *client),
		leave:   make(chan *client),
		clients: make(map[*client]bool),
	}
}

func (r *Room) Run() {
	for {
		select {
		case client := <-r.join:
			// joining
			r.clients[client] = true
			log.Println("AHRSWeb: New client joined")
		case client := <-r.leave:
			// leaving
			delete(r.clients, client)
			close(client.send)
			log.Println("AHRSWeb: Client left")
		case msg := <-r.forward:
			log.Println("AHRSWeb: Message received")
			// forward message to all clients
			for client := range r.clients {
				select {
				case client.send <- msg:
					log.Print(" -- sent to client")
				default:
					log.Print(" -- couldn't send to client")
				}
			}
		}
	}
}

const (
	socketBufferSize  = 1024
	messageBufferSize = 10
)

var upgrader = &websocket.Upgrader{ReadBufferSize: socketBufferSize, WriteBufferSize: socketBufferSize}

func (r *Room) ServeHTTP(w http.ResponseWriter, req *http.Request) {
	socket, err := upgrader.Upgrade(w, req, nil)
	if err != nil {
		log.Fatal("ServeHTTP:", err)
		return
	}
	client := &client{
		socket: socket,
		send:   make(chan []byte, messageBufferSize),
		room:   r,
	}
	r.join <- client
	defer func() { r.leave <- client }()
	go client.write()
	client.read()
}
