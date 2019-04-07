// Lightweight minimal matrix algebra functions for 2-D and 3-D matrices
package magkal

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

func matDiff(a, b [][]float64) (x[][]float64) {
	x = make([][]float64, len(a))
	for i:=0; i<len(a); i++ {
		x[i] = make([]float64, len(a[0]))
		for j := 0; j < len(b[0]); j++ {
			x[i][j] = a[i][j] - b[i][j]
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

func matInverse(a [][]float64) (x [][]float64) {
	// Works only for 2D and 3D matrices
	if len(a)==1 {
		return [][]float64{{1/a[0][0]}}
	} else if len(a)==2 {
		det := a[0][0]*a[1][1]-a[0][1]*a[1][0]
		if det==0 {
			return nil
		}
		return matSMul(1/det, [][]float64{{a[1][1], -a[0][1]}, {-a[1][0], a[0][0]}})
	} else if len(a)==3 {
		det :=
			a[0][0]*(a[1][1]*a[2][2]-a[1][2]*a[2][1]) +
				a[0][1]*(a[2][0]*a[1][2]-a[1][0]*a[2][2]) +
				a[0][2]*(a[2][1]*a[1][0]-a[1][1]*a[2][0])
		if det==0 {
			return nil
		}
		return matSMul(1/det, [][]float64{
			{a[1][1]*a[2][2]-a[1][2]*a[2][1], a[0][2]*a[2][1]-a[0][1]*a[2][2], a[0][1]*a[1][2]-a[1][1]*a[0][2]},
			{a[2][0]*a[1][2]-a[1][0]*a[2][2], a[0][0]*a[2][2]-a[0][2]*a[2][0], a[0][2]*a[1][0]-a[0][0]*a[1][2]},
			{a[2][1]*a[1][0]-a[1][1]*a[2][0], a[2][0]*a[0][1]-a[0][0]*a[2][1], a[0][0]*a[1][1]-a[0][1]*a[1][0]},
		})
	} else {
		return nil
	}
}
