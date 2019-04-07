// The Kalman computation uses a Kalman Filter to estimate the K and L.
// It works by using the measurement of the magnitude of the magnetic field
package magkal

import (
	"fmt"

	"../ahrs"
)

const (
	magNoise     = 0.1 // Typical magnetometer noise scale TODO measure as it comes in
	kUncertainty = 0.2 // Initial uncertainty of K
	lUncertainty = 0.5 // Initial uncertainty of L
	processNoise = 0.1 // Amount by which K, L vary on a daily timescale, should be small
)

type MagKalStateKalman struct {
	MagKalState
	x, p, q, r, s, u, kk, nHat, h [][]float64
	y                             float64
}

func ComputeKalman(nn MagKalState, cIn chan ahrs.Measurement, cOut chan MagKalState) {
	n := new(MagKalStateKalman)
	n.MagKalState = nn

	if NormVec(n.K) < Small {
		n.K = [3]float64{1, 1, 1}
		n.L = [3]float64{0, 0, 0}
	}

	// Initialize the Kalman state x
	// This is just {K1,L1,K2,L2,K3,L3}
	n.x = [][]float64{{n.K[0]}, {n.L[0]/AvgMagField}, {n.K[1]}, {n.L[1]/AvgMagField}, {n.K[2]}, {n.L[2]/AvgMagField}}

	// Initialize the Kalman uncertainty P and process noise Q
	n.p = make([][]float64, 6)
	n.q = make([][]float64, 6)
	for i:=0; i<3; i++ {
		n.p[2*i] = make([]float64, 6)
		n.p[2*i+1] = make([]float64, 6)
		n.p[2*i][2*i] = kUncertainty * kUncertainty
		n.p[2*i+1][2*i+1] = lUncertainty * lUncertainty

		n.q[2*i] = make([]float64, 6)
		n.q[2*i+1] = make([]float64, 6)
		n.q[2*i][2*i] = processNoise * processNoise / 86400
		n.q[2*i+1][2*i+1] = processNoise * processNoise  / 86400
	}

	n.r = [][]float64{{magNoise * magNoise}}

	n.h = make([][]float64, 1) // Kalman noise Jacobian
	n.h[0] = make([]float64, 6)
	id := matIdentity(6)

	for m := range cIn { // Receive input measurements
		n.u = [][]float64{{m.M1/AvgMagField}, {m.M2/AvgMagField}, {m.M3/AvgMagField}}

		// Calculate estimated measurement
		n.nHat = calcMagField(n.x, n.u)

		// No evolution for x

		// Evolve p
		for i:=0; i<6; i++ {
			for j:=0; j<6; j++ {
				n.p[i][j] += n.q[i][j]
			}
		}

		// Calculate measurement residual
		n.y = 1 // In natural units
		for i:=0; i<3; i++ {
			n.y -= n.nHat[i][0]*n.nHat[i][0]
		}

		// Calculate Jacobian
		for i:=0; i<3; i++ {
			n.h[0][2*i] = 2*n.nHat[i][0]*n.u[i][0]
			n.h[0][2*i+1] = 2*n.nHat[i][0]
		}

		// Calculate S
		n.s = matAdd(n.r, matMul(n.h, matMul(n.p, matTranspose(n.h))))

		// Kalman Gain
		n.kk = matSMul(1/n.s[0][0], matMul(n.p, matTranspose(n.h)))

		// State correction
		n.x = matAdd(n.x, matSMul(n.y, n.kk))

		// State covariance correction
		n.p = matMul(matAdd(id, matSMul(-1, matMul(n.kk, n.h))), n.p)

		// Copy all the internal values to the MagKalState
		n.T = m.T
		n.K = [3]float64{n.x[0][0], n.x[2][0], n.x[4][0]}
		n.L = [3]float64{n.x[1][0]*AvgMagField, n.x[3][0]*AvgMagField, n.x[5][0]*AvgMagField}
		n.updateLogMap(&m, n.LogMap)
		n.updateKalmanLogMap()

		cOut <- n.MagKalState // Send results when requested, blocking
	}

	close(cOut) // When cIn is closed, close cOut
}

func (n *MagKalStateKalman) updateKalmanLogMap() {
	n.LogMap["r"] = n.r[0][0]
	n.LogMap["s"] = n.s[0][0]
	for i:=0; i<3; i++ {
		n.LogMap[fmt.Sprintf("u%d", i+1)] = n.u[i][0]
		n.LogMap[fmt.Sprintf("nHat%d", i+1)] = n.nHat[i][0]
		n.LogMap[fmt.Sprintf("k%d", i+1)] = n.x[2*i][0]
		n.LogMap[fmt.Sprintf("l%d", i+1)] = n.x[2*i+1][0]
		n.LogMap[fmt.Sprintf("hk%d", i+1)] = n.h[0][2*i]
		n.LogMap[fmt.Sprintf("hl%d", i+1)] = n.h[0][2*i+1]
		n.LogMap[fmt.Sprintf("kkk%d", i+1)] = n.kk[2*i][0]
		n.LogMap[fmt.Sprintf("kkl%d", i+1)] = n.kk[2*i+1][0]
		for j:=0; j<3; j++ {
			n.LogMap[fmt.Sprintf("pk%dk%d", i+1, j+1)] = n.p[2*i][2*j]
			n.LogMap[fmt.Sprintf("pk%dl%d", i+1, j+1)] = n.p[2*i][2*j+1]
			n.LogMap[fmt.Sprintf("pl%dk%d", i+1, j+1)] = n.p[2*i+1][2*j]
			n.LogMap[fmt.Sprintf("pl%dl%d", i+1, j+1)] = n.p[2*i+1][2*j+1]
			n.LogMap[fmt.Sprintf("qk%dk%d", i+1, j+1)] = n.p[2*i][2*j]
			n.LogMap[fmt.Sprintf("qk%dl%d", i+1, j+1)] = n.p[2*i][2*j+1]
			n.LogMap[fmt.Sprintf("ql%dk%d", i+1, j+1)] = n.p[2*i+1][2*j]
			n.LogMap[fmt.Sprintf("ql%dl%d", i+1, j+1)] = n.p[2*i+1][2*j+1]
		}
	}
}

func (n *MagKalStateKalman) updateKalmanLogMap2() {
	for i:=0; i<6; i++ {
		n.LogMap[fmt.Sprintf("x%d", i)] = n.x[i][0]
		n.LogMap[fmt.Sprintf("h%d", i)] = n.h[0][i]
		n.LogMap[fmt.Sprintf("kk%d", i)] = n.kk[i][0]
		for j:=0; j<6; j++ {
			n.LogMap[fmt.Sprintf("p%d%d", i)] = n.p[i][j]
			n.LogMap[fmt.Sprintf("q%d%d", i)] = n.p[i][j]
		}
	}
	n.LogMap["r"] = n.r[0][0]
	n.LogMap["s"] = n.s[0][0]
	for i:=0; i<3; i++ {
		n.LogMap[fmt.Sprintf("u%d", i)] = n.u[i][0]
		n.LogMap[fmt.Sprintf("nHat%d", i)] = n.nHat[i][0]
	}
}

func calcMagField(x, u [][]float64) (n [][]float64) {
	n = make([][]float64, len(u))
	for i:=0; i<len(u); i++ {
		n[i] = []float64{x[2*i][0]*u[i][0] + x[2*i+1][0]}
	}
	return n
}

// Lightweight minimal matrix algebra functions
func matAdd(a, b [][]float64) (x[][]float64) {
	x = make([][]float64, len(a))
	for i:=0; i<len(a); i++ {
		x[i] = make([]float64, len(a[0]))
		for j := 0; j < len(b[0]); j++ {
			x[i][j] = a[i][j] + b[i][j]
		}
	}
	return x
}

func matSMul(k float64, a [][]float64) (x [][]float64) {
	x = make([][]float64, len(a))
	for i:=0; i<len(a); i++ {
		x[i] = make([]float64, len(a[0]))
		for j:=0; j<len(a[0]); j++ {
			x[i][j] = k*a[i][j]
		}
	}
	return x
}

func matMul(a, b [][]float64) (x [][]float64) {
	x = make([][]float64, len(a))
	for i:=0; i<len(a); i++ {
		x[i] = make([]float64, len(b[0]))
		for j:=0; j<len(b[0]); j++ {
			for k:=0; k<len(b); k++ {
				x[i][j] += a[i][k]*b[k][j]
			}
		}
	}
	return x
}

func matTranspose(a [][]float64) (x [][]float64) {
	x = make([][]float64, len(a[0]))
	for i:=0; i<len(x); i++ {
		x[i] = make([]float64, len(a))
		for j:=0; j<len(x[0]); j++ {
			x[i][j] = a[j][i]
		}
	}
	return x
}

func matIdentity(n int) (id [][]float64) {
	id = make([][]float64, n) // Identity matrix

	for i:=0; i<n; i++ {
		id[i] = make([]float64, n)
		id[i][i] = 1
	}
	return
}
