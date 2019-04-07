package magkal

import "testing"

const eps = 1e-6

func matDifferent(a, b [][]float64) (bool) {
	for i:=0; i<len(a); i++ {
		for j:=0; j<len(a); j++ {
			if a[i][j]-b[i][j]>eps || b[i][j]-a[i][j]>eps {
				return true
			}
		}
	}
	return false
}

func TestMatInv1D(t *testing.T) {
	a := [][]float64{{2}}
	b := matInverse(a)
	if matDifferent(matMul(a, b), matIdentity(len(a))) {
		t.Error("1D matrix doesn't invert correctly")
	}
}

func TestMatInv2D(t *testing.T) {
	a := [][]float64{{2, 2}, {2, 3}}
	if matDifferent(matMul(a, matInverse(a)), matIdentity(len(a))) {
		t.Error("2D matrix doesn't invert correctly")
	}
}

func TestMatInv3D(t *testing.T) {
	a := [][]float64{{0, -6, -4}, {2, -8, -4}, {-6, 8, 2}}
	if matDifferent(matMul(a, matInverse(a)), matIdentity(len(a))) {
		t.Error("3D matrix doesn't invert correctly")
	}
}


