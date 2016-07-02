package main

import (
	"fmt"
	"math"
	"github.com/westphae/quaternion"
	"testing"
)

const pi = math.Pi

func TestRoundTrips(t *testing.T) {
	phis := []float64{0, 0.1, 0.2, 0.5, 1, 1.5, 2, 2.5, 3, -3, -2, -1, -0.5, -0.2}
	thetas := []float64{0.1, 0.2, 0.5, 1, 1.5, -1.5, -0.5, -0.2, 0.2, 0.1, -1, -0.5, -0.2, 0}
	psis := []float64{1, 1.5, 2, 2.5, 3, 4, 0.1, 0.2, 0.5, 5, 5.5, 3.5, 6, 0}
	var q0, q1, q2, q3 float64
	var phi, theta, psi float64
	var phiOut, thetaOut, psiOut float64

	for i := 0; i < len(phis); i++ {
		phi = phis[i]
		theta = thetas[i]
		psi = psis[i]
		q0, q1, q2, q3 = ToQuaternion(phi, theta, psi)
		phiOut, thetaOut, psiOut = FromQuaternion(q0, q1, q2, q3)
		if math.Abs(phi-phiOut) > 1e-6 || math.Abs(theta-thetaOut) > 1e-6 || math.Abs(psi-psiOut) > 1e-6 {
			fmt.Printf("%+5.3f -> %+5.3f, %+5.3f -> %+5.3f, %+5.3f -> %+5.3f\n",
				phi, phiOut, theta, thetaOut, psi, psiOut)
			t.Fail()
		}

	}
}

func TestSpecificToQuaternion(t *testing.T) {
	const (
		c30 = math.Sqrt(3)/2
		c60 = 0.5
	)
	phis :=   []float64{0,    0,  0,     0,       0,    0, pi/3, pi/3}
	thetas := []float64{0,    0,  0,     0,       0, pi/3,    0,    0}
	psis :=   []float64{0, pi/2, pi, 4*pi/3, 3*pi/2, pi/2, pi/2,    0}
	w1s :=    []float64{0,    1,  0,   -c30,     -1,  c60,    1,    0}
	w2s :=    []float64{1,    0, -1,   -c60,      0,    0,    0,    1}
	w3s :=    []float64{0,    0,  0,      0,      0,  c30,    0,    0}

	u := quaternion.Quaternion{0, 1, 0, 0}
	var (
		e0, e1, e2, e3 float64
		w, e, x		quaternion.Quaternion
	)

	for i := 0; i < len(phis); i++ {
		w = quaternion.Quaternion{0, w1s[i], w2s[i], w3s[i]}
		e0, e1, e2, e3 = ToQuaternion(phis[i], thetas[i], psis[i])
		e = quaternion.Quaternion{e0, e1, e2, e3}
		x = quaternion.Prod(quaternion.Conj(e), u, e)

		if math.Abs(w.W-x.W) > 1e-6 || math.Abs(w.X-x.X) > 1e-6 ||
				math.Abs(w.Y-x.Y) > 1e-6 || math.Abs(w.Z-x.Z) > 1e-6 {
			fmt.Println(i)
			fmt.Println(e)
			fmt.Printf("%+5.3f -> %+5.3f, %+5.3f -> %+5.3f, %+5.3f -> %+5.3f, %+5.3f -> %+5.3f\n",
				w.W, x.W, w.X, x.X, w.Y, x.Y, w.Z, x.Z)
			t.Fail()
		}

	}
}
