// +build !arm

package main

type MPUListener struct {
}

func (ml *MPUListener) SetRoom(r *room) {
}

func (ml *MPUListener) Init() {
}

func (ml *MPUListener) Close() {
}

func (ml *MPUListener) GetData() *AHRSData {
	return nil
}

func (ml *MPUListener) Run() {
}
