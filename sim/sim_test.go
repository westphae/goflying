package main

import (
	"testing"
	"fmt"
	"math"
)

func TestToFromQuaternionSimple(t *testing.T) {
	phi, theta, psi := FromQuaternion(0.5, 0.5, 0.5, 0.5)
	q0, q1, q2, q3 := ToQuaternion(phi, theta, psi)
	if math.Abs(q0-0.5) > 1e-6 || math.Abs(q1-0.5) > 1e-6 || math.Abs(q2-0.5) > 1e-6 || math.Abs(q3-0.5) > 1e-6 {
		t.Fail()
	}
}

func TestRoundTrips(t *testing.T) {
	phis  := []float64{0, 0.1, 0.2, 0.5, 1, 1.5, 2, 2.5, 3, -3, -2, -1, -0.5, -0.2}
	thetas := []float64{0.1, 0.2, 0.5, 1, 1.5, -1.5, -0.5, -0.2, 0.2, 0.1, -1, -0.5, -0.2, 0}
	psis := []float64{1, 1.5, 2, 2.5, 3, 4, 0.1, 0.2, 0.5, 5, 5.5, 3.5, 6, 0}
	var q0, q1, q2, q3 float64
	var phi, theta, psi float64
	var phiOut, thetaOut, psiOut float64

	for i:=0; i < len(phis); i++ {
		phi = phis[i]
		theta = thetas[i]
		psi = psis[i]
		q0, q1, q2, q3 = ToQuaternion(phi, theta, psi)
		phiOut, thetaOut, psiOut = FromQuaternion(q0, q1, q2, q3)
		if math.Abs(phi-phiOut) > 1e-6 || math.Abs(theta-thetaOut) > 1e-6 || math.Abs(psi-psiOut) > 1e-6 {
			fmt.Printf("%+5.3f -> %+5.3f, %+5.3f -> %+5.3f, %5+.3f -> %+5.3f\n",
				phi, phiOut, theta, thetaOut, psi, psiOut)
			t.Fail()
		}

	}
}
