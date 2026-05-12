module github.com/westphae/goflying

go 1.19

require (
	github.com/gorilla/websocket v1.5.3
	github.com/kidoman/embd v0.0.0-20170508013040-d3d8c0c5c68d
	github.com/skelterjohn/go.matrix v0.0.0-20130517144113-daa59528eefd
	github.com/westphae/quaternion v0.0.0-20210908005042-fa06d546065c
)

require github.com/golang/glog v1.2.5 // indirect

// kidoman/embd has been unmaintained since 2017; its kernel-version parser
// (detect.go parseVersion) panics on modern Raspberry Pi OS kernel strings
// like "6.12.62+rpt-rpi-v8" because it can't handle the "+rpt" suffix on the
// patch component. westphae/embd has a small patch. See the "Deferred
// modernizations" section of CLAUDE.md for the periph.io migration that
// should eventually remove this entirely.
replace github.com/kidoman/embd => github.com/westphae/embd v0.1.0
