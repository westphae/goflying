// +build !arm

package main

type MPU9250Listener struct {
}

func (ml *MPU9250Listener) SetRoom(r *room) {
}

func (ml *MPU9250Listener) Init() {
}

func (ml *MPU9250Listener) Close() {
}

func (ml *MPU9250Listener) GetData() *AHRSData {
	return nil
}

func (ml *MPU9250Listener) Run() {
}
