// The Kalman computation uses a Kalman Filter to estimate the K and L.
// It works by using the measurement of the magnitude of the magnetic field
package magkal

import "../ahrs"

const (
	NSigma = 0.1     // Initial uncertainty scale
	Epsilon = 1e-2   // Some tiny noise scale
)

func ComputeKalman(n MagKalState, cIn chan ahrs.Measurement, cOut chan MagKalState) {
	if NormVec(n.K) < Small {
		n.K = [3]float64{1, 1, 1}
		n.L = [3]float64{0, 0, 0}
	}

	// Initialize the Kalman state x
	// This is just {K1,L1,K2,L2,K3,L3}
	x := [][]float64{{n.K[0]}, {n.L[0]/AvgMagField}, {n.K[1]}, {n.L[1]/AvgMagField}, {n.K[2]}, {n.L[2]/AvgMagField}}

	// Initialize the Kalman uncertainty P and process noise Q
	p := make([][]float64, 6)
	q := make([][]float64, 6)
	for i:=0; i<3; i++ {
		p[2*i] = make([]float64, 6)
		p[2*i+1] = make([]float64, 6)
		p[2*i][2*i] = NSigma * NSigma
		p[2*i+1][2*i+1] = NSigma * NSigma

		q[2*i] = make([]float64, 6)
		q[2*i+1] = make([]float64, 6)
		q[2*i][2*i] = Epsilon * Epsilon / (86400*10)
		q[2*i+1][2*i+1] = Epsilon * Epsilon  / (86400*10)
	}

	r := [][]float64{{Epsilon*Epsilon}}

	var (
		y              float64 // Kalman measurement error
		s, u, kk, nHat [][]float64
	)

	h := make([][]float64, 1) // Kalman noise Jacobian
	h[0] = make([]float64, 6)
	id := make([][]float64, 6) // Identity matrix

	for i:=0; i<6; i++ {
		id[i] = make([]float64, 6)
		id[i][i] = 1
	}

	for m := range cIn { // Receive input measurements
		u = [][]float64{{m.M1/AvgMagField}, {m.M2/AvgMagField}, {m.M3/AvgMagField}}

		// Calculate estimated measurement
		nHat = calcMagField(x, u)

		// No evolution for x

		// Evolve p
		for i:=0; i<6; i++ {
			for j:=0; j<6; j++ {
				p[i][j] += q[i][j]
			}
		}

		// Calculate measurement residual
		y = 1 // In natural units
		for i:=0; i<3; i++ {
			y -= nHat[i][0]*nHat[i][0]
		}

		// Calculate Jacobian
		for i:=0; i<3; i++ {
			h[0][2*i] = 2*nHat[i][0]*u[i][0]
			h[0][2*i+1] = 2 * nHat[i][0]
		}

		// Calculate S
		s = matAdd(r, matMul(h, matMul(p, matTranspose(h))))

		// Kalman Gain
		kk = matSMul(1/s[0][0], matMul(p, matTranspose(h)))

		// State correction
		x = matAdd(x, matSMul(y, kk))

		// State covariance correction
		p = matMul(matAdd(id, matSMul(-1, matMul(kk, h))), p)

		// Copy all the internal values to the MagKalState
		n.T = m.T
		n.K = [3]float64{x[0][0], x[2][0], x[4][0]}
		n.L = [3]float64{x[1][0]*AvgMagField, x[3][0]*AvgMagField, x[5][0]*AvgMagField}
		n.updateLogMap(&m, n.LogMap)

		select {
		case cOut <- n: // Send results when requested, non-blocking
		default:
		}
	}

	close(cOut) // When cIn is closed, close cOut
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
