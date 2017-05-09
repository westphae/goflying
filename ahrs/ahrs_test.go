package ahrs

import (
	"github.com/skelterjohn/go.matrix"
	"log"
	"math"
	"math/rand"
	"testing"
	"time"
)

func createRandomState() (s *KalmanState) {
	s = &KalmanState{State{
		U1: rand.Float64()*100 + 15,
		U2: rand.Float64()*10 - 5,
		U3: rand.Float64()*10 - 5,
		Z1: rand.Float64()*1 - 0.5,
		Z2: rand.Float64()*1 - 0.5,
		Z3: rand.Float64()*1 - 0.5,
		E0: rand.Float64()*2 - 1,
		E1: rand.Float64()*2 - 1,
		E2: rand.Float64()*2 - 1,
		E3: rand.Float64()*2 - 1,
		H1: rand.Float64()*20 - 10,
		H2: rand.Float64()*20 - 10,
		H3: rand.Float64()*20 - 10,
		N1: rand.Float64()*20 - 10,
		N2: rand.Float64()*20 - 10,
		N3: rand.Float64()*20 - 10,

		V1: rand.Float64()*20 - 10,
		V2: rand.Float64()*20 - 10,
		V3: rand.Float64()*10 - 5,
		C1: rand.Float64()*0.1 - 0.05,
		C2: rand.Float64()*0.1 - 0.05,
		C3: rand.Float64()*0.1 - 0.05,
		F0: rand.Float64()*2 - 1,
		F1: rand.Float64()*2 - 1,
		F2: rand.Float64()*2 - 1,
		F3: rand.Float64()*2 - 1,
		D1: rand.Float64()*0.1 - 0.05,
		D2: rand.Float64()*0.1 - 0.05,
		D3: rand.Float64()*0.1 - 0.05,
		L1: rand.Float64()*1 - 0.5,
		L2: rand.Float64()*1 - 0.5,
		L3: rand.Float64()*1 - 0.5,

		T: 10,
		M: matrix.Zeros(32, 32),
		N: matrix.Zeros(32, 32),
	}}

	s.normalize()

	return
}

func stateMap(s *KalmanState) map[int]*float64 {
	return map[int]*float64{
		0:  &s.U1,
		1:  &s.U2,
		2:  &s.U3,
		3:  &s.Z1,
		4:  &s.Z2,
		5:  &s.Z3,
		6:  &s.E0,
		7:  &s.E1,
		8:  &s.E2,
		9:  &s.E3,
		10: &s.H1,
		11: &s.H2,
		12: &s.H3,
		13: &s.N1,
		14: &s.N2,
		15: &s.N3,
		16: &s.V1,
		17: &s.V2,
		18: &s.V3,
		19: &s.C1,
		20: &s.C2,
		21: &s.C3,
		22: &s.F0,
		23: &s.F1,
		24: &s.F2,
		25: &s.F3,
		26: &s.D1,
		27: &s.D2,
		28: &s.D3,
		29: &s.L1,
		30: &s.L2,
		31: &s.L3,
	}
}

func measMap(m *Measurement) map[int]*float64 {
	return map[int]*float64{
		0:  &m.U1,
		1:  &m.U2,
		2:  &m.U3,
		3:  &m.W1,
		4:  &m.W2,
		5:  &m.W3,
		6:  &m.A1,
		7:  &m.A2,
		8:  &m.A3,
		9:  &m.B1,
		10: &m.B2,
		11: &m.B3,
		12: &m.M1,
		13: &m.M2,
		14: &m.M3,
	}
}

func TestJacobianMeasurement(t *testing.T) {
	for n := 0; n < 100; n++ {
		rand.Seed(time.Now().Unix())
		s := createRandomState()
		smap := stateMap(s)

		m := s.PredictMeasurement()
		mmap := measMap(m)

		h := s.calcJacobianMeasurement()

		for i := 0; i < 32; i++ {
			//TODO westphae: don't skip these after working out Jacobian for magnetometer
			if (i >= 12 && i <= 14) || (i >= 29) {
				continue
			}
			*(smap[i]) += Small
			s.calcRotationMatrices()
			mm := s.PredictMeasurement()
			*(smap[i]) -= Small
			s.calcRotationMatrices()
			mmmap := measMap(mm)
			//TODO westphae: indices all the way up to 15 after working out Jacobian for magnetometer
			for j := 0; j < 12; j++ {
				dM := (*(mmmap[j]) - *(mmap[j])) / Small
				if math.Abs(dM-h.Get(j, i)) > 1e-4 {
					log.Printf("Error in index %2d,%2d: Calc %6f, Jacobian was %6f\n", j, i, dM, h.Get(j, i))
					t.Fail()
				}
			}
		}
	}
}

func TestJacobianState(t *testing.T) {
	//TODO westphae: loop over 100, re-seed
	for n := 0; n < 1; n++ {
		//rand.Seed(time.Now().Unix())
		rand.Seed(5)
		s := createRandomState()
		t1 := s.T + 1e6*Small

		f := s.calcJacobianState(t1)

		s1 := *s // Shallow copy
		s1.Predict(t1)
		smap := stateMap(&s1)

		for i := 0; i < 32; i++ {
			//TODO westphae: don't skip these after working out Jacobian
			if (i >= 12 && i <= 14) || (i >= 29) {
				continue
			}
			ss := *s // Shallow copy
			ssmap := stateMap(&ss)
			*(ssmap[i]) += Small
			if i >= 6 && i <= 9 {
				r := math.Sqrt((1 - *(ssmap[i])**(ssmap[i])) / (1 - (*(ssmap[i])-Small)*(*(ssmap[i])-Small)))
				for j := 6; j <= 9; j++ {
					if j != i {
						*(ssmap[j]) *= r
					}
				}
			}

			ss.Predict(t1)

			for j := 0; j < 32; j++ {
				//TODO westphae: don't skip these after working out Jacobian
				if (j >= 12 && j <= 14) || (j >= 29) {
					continue
				}
				dS := (*(ssmap[j]) - *(smap[j])) / Small
				if math.Abs(dS-f.Get(j, i)) > 1e-4 {
					log.Printf("Error in index %2d,%2d: Calc %6f, Jacobian was %6f\n", j, i, dS, f.Get(j, i))
					t.Fail()
					//} else if math.Abs(dS) > Small {
					//	log.Printf("Passed   index %2d,%2d: Calc %6f, Jacobian was %6f\n", j, i, dS, f.Get(j, i))
				}
			}
		}
	}
}

func TestAccumulator(t *testing.T) {
	const Decay = 0.995

	var n, m, v float64

	rand.Seed(time.Now().Unix())

	N := 1 / (1 - Decay) // Go long enough to get good statistics
	a := NewVarianceAccumulator(0, 0, Decay)
	x := 0.0
	for i := 1; i < int(50*N); i++ {
		x += 1 + rand.NormFloat64()
		n, m, v = a(x)
	}
	if math.Abs(n-N) > 0.01 {
		log.Printf("Error: effective observations was %6f, should be %6f\n", n, N)
		t.Fail()
	}
	if math.Abs(m-1) > 2/math.Sqrt(N) {
		log.Printf("Error: mean was %6f, should be 1\n", m)
		t.Fail()
	}
	if math.Abs(v-1) > 2/math.Sqrt(N) {
		log.Printf("Error: var was %6f, should be 1\n", v)
		t.Fail()
	}
	log.Printf("Success: n=%6.0f, m=%6f, v=%6f\n", n, m, v)
}

func TestMakeUnitVector(t *testing.T) {
	var v, w *[3]float64

	rand.Seed(time.Now().Unix())

	v = new([3]float64)
	for i := 0; i < 3; i++ {
		v[i] = 2*rand.Float64() - 1
	}

	w, _ = MakeUnitVector(*v)

	r := math.Sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])
	s := math.Sqrt(w[0]*w[0] + w[1]*w[1] + w[2]*w[2])

	if math.Abs(s-1) > Small {
		log.Printf("Error: returned vector is not unit size, was %6f\n", s)
		t.Fail()
	}

	if math.Abs(v[0]*w[0] + v[1]*w[1] + v[2]*w[2] - r*s) > Small {
		log.Println("Error: returned vector is not parallel to input")
		t.Fail()
	}
}

func TestMakeOrthogonal(t *testing.T) {
	var u, v, w *[3]float64

	rand.Seed(time.Now().Unix())

	u = new([3]float64)
	for i := 0; i < 3; i++ {
		u[i] = 2*rand.Float64() - 1
	}
	u, _ = MakeUnitVector(*u)

	v = new([3]float64)
	for i := 0; i < 3; i++ {
		v[i] = 2*rand.Float64() - 1
	}
	v, _ = MakeUnitVector(*v)

	w = MakeOrthogonal(*u, *v)

	if math.Abs(w[0]*v[0] + w[1]*v[1] + w[2]*v[2]) > Small {
		log.Println("Error: out vector is not perpendicular to ortho vector")
		log.Printf("u: %3f %3f %3f\n", u[0], u[1], u[2])
		log.Printf("v: %3f %3f %3f\n", v[0], v[1], v[2])
		log.Printf("w: %3f %3f %3f\n", w[0], w[1], w[2])
		t.Fail()
	}
}

func TestMakePerpendicular(t *testing.T) {
	var u, v, w *[3]float64

	rand.Seed(time.Now().Unix())

	u = new([3]float64)
	for i := 0; i < 3; i++ {
		u[i] = 2*rand.Float64() - 1
	}
	u, _ = MakeUnitVector(*u)

	v = new([3]float64)
	for i := 0; i < 3; i++ {
		v[i] = 2*rand.Float64() - 1
	}
	v, _ = MakeUnitVector(*v)

	w, _ = MakePerpendicular(*u, *v)

	if math.Abs(w[0]*u[0] + w[1]*u[1] + w[2]*u[2]) > Small {
		log.Println("Error: out vector is not perpendicular to first vector")
		log.Printf("u: %3f %3f %3f\n", u[0], u[1], u[2])
		log.Printf("v: %3f %3f %3f\n", v[0], v[1], v[2])
		log.Printf("w: %3f %3f %3f\n", w[0], w[1], w[2])
		t.Fail()
	}

	if math.Abs(w[0]*v[0] + w[1]*v[1] + w[2]*v[2]) > Small {
		log.Println("Error: out vector is not perpendicular to second vector")
		log.Printf("u: %3f %3f %3f\n", u[0], u[1], u[2])
		log.Printf("v: %3f %3f %3f\n", v[0], v[1], v[2])
		log.Printf("w: %3f %3f %3f\n", w[0], w[1], w[2])
		t.Fail()
	}
}

func TestMakeHardSoftRotationMatrix(t *testing.T) {
	var (
		h1, h2, s1, s2, z1, z2, out *[3]float64
		w                           *[3][3]float64
	)

	rand.Seed(time.Now().Unix())

	h1 = new([3]float64)
	for i := 0; i < 3; i++ {
		h1[i] = 2*rand.Float64() - 1
	}
	h1, _ = MakeUnitVector(*h1)

	h2 = new([3]float64)
	for i := 0; i < 3; i++ {
		h2[i] = 2*rand.Float64() - 1
	}
	h2, _ = MakeUnitVector(*h2)

	s1 = new([3]float64)
	for i := 0; i < 3; i++ {
		s1[i] = 2*rand.Float64() - 1
	}
	s1, _ = MakeUnitVector(*s1)

	s2 = new([3]float64)
	for i := 0; i < 3; i++ {
		s2[i] = 2*rand.Float64() - 1
	}
	s2, _ = MakeUnitVector(*s2)

	w, _ = MakeHardSoftRotationMatrix(*h1, *s1, *h2, *s2)

	z1, _ = MakePerpendicular(*h1, *s1)
	z2, _ = MakePerpendicular(*h2, *s2)

	s1, _ = MakePerpendicular(*h1, *z1)
	s2, _ = MakePerpendicular(*h2, *z2)

	out = new([3]float64)
	out[0] = w[0][0]*h1[0] + w[0][1]*h1[1] + w[0][2]*h1[2]
	out[1] = w[1][0]*h1[0] + w[1][1]*h1[1] + w[1][2]*h1[2]
	out[2] = w[2][0]*h1[0] + w[2][1]*h1[1] + w[2][2]*h1[2]
	if math.Abs(out[0]-h2[0]) + math.Abs(out[1]-h2[1]) + math.Abs(out[2]-h2[2]) > Small {
		log.Println("Error: out vector is not mapped to first hard vector")
		log.Printf("h2:  %3f %3f %3f\n", h2[0], h2[1], h2[2])
		log.Printf("out: %3f %3f %3f\n", out[0], out[1], out[2])
		t.Fail()
	}

	out[0] = w[0][0]*z1[0] + w[0][1]*z1[1] + w[0][2]*z1[2]
	out[1] = w[1][0]*z1[0] + w[1][1]*z1[1] + w[1][2]*z1[2]
	out[2] = w[2][0]*z1[0] + w[2][1]*z1[1] + w[2][2]*z1[2]
	if math.Abs(out[0]-z2[0]) + math.Abs(out[1]-z2[1]) + math.Abs(out[2]-z2[2]) > Small {
		log.Println("Error: out vector is not mapped to perpendicular vector")
		log.Printf("z2:  %3f %3f %3f\n", z2[0], z2[1], z2[2])
		log.Printf("out: %3f %3f %3f\n", out[0], out[1], out[2])
		t.Fail()
	}

	out[0] = w[0][0]*s1[0] + w[0][1]*s1[1] + w[0][2]*s1[2]
	out[1] = w[1][0]*s1[0] + w[1][1]*s1[1] + w[1][2]*s1[2]
	out[2] = w[2][0]*s1[0] + w[2][1]*s1[1] + w[2][2]*s1[2]
	if math.Abs(out[0]-s2[0]) + math.Abs(out[1]-s2[1]) + math.Abs(out[2]-s2[2]) > Small {
		log.Println("Error: out vector is not mapped to perpendicular vector")
		log.Printf("s2:  %3f %3f %3f\n", s2[0], s2[1], s2[2])
		log.Printf("out: %3f %3f %3f\n", out[0], out[1], out[2])
		t.Fail()
	}
}
