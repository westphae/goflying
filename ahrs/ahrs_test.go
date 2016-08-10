package ahrs

import (
	"log"
	"math"
	"math/rand"
	"testing"
	"time"
	"github.com/skelterjohn/go.matrix"
)

func createRandomState() (s *State) {
	s = &State{
		U1: rand.Float64()*100+15,
		U2: rand.Float64()*10-5,
		U3: rand.Float64()*10-5,
		Z1: rand.Float64()*1-0.5,
		Z2: rand.Float64()*1-0.5,
		Z3: rand.Float64()*1-0.5,
		E0: rand.Float64()*2-1,
		E1: rand.Float64()*2-1,
		E2: rand.Float64()*2-1,
		E3: rand.Float64()*2-1,
		H1: rand.Float64()*20-10,
		H2: rand.Float64()*20-10,
		H3: rand.Float64()*20-10,
		N1: rand.Float64()*20-10,
		N2: rand.Float64()*20-10,
		N3: rand.Float64()*20-10,

		V1: rand.Float64()*20-10,
		V2: rand.Float64()*20-10,
		V3: rand.Float64()*10-5,
		C1: rand.Float64()*0.1-0.05,
		C2: rand.Float64()*0.1-0.05,
		C3: rand.Float64()*0.1-0.05,
		F0: rand.Float64()*2-1,
		F1: rand.Float64()*2-1,
		F2: rand.Float64()*2-1,
		F3: rand.Float64()*2-1,
		D1: rand.Float64()*0.1-0.05,
		D2: rand.Float64()*0.1-0.05,
		D3: rand.Float64()*0.1-0.05,
		L1: rand.Float64()*1-0.5,
		L2: rand.Float64()*1-0.5,
		L3: rand.Float64()*1-0.5,

		T : 10,
		M : matrix.Zeros(32, 32),
		N : matrix.Zeros(32, 32),
	}

	s.normalize()

	return
}

func stateMap(s *State) (map[int]*float64) {
	return map[int]*float64{
		 0: &s.U1,
		 1: &s.U2,
		 2: &s.U3,
		 3: &s.Z1,
		 4: &s.Z2,
		 5: &s.Z3,
		 6: &s.E0,
		 7: &s.E1,
		 8: &s.E2,
		 9: &s.E3,
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

func measMap(m *Measurement) (map[int]*float64) {
	return map[int]*float64{
		 0: &m.U1,
		 1: &m.U2,
		 2: &m.U3,
		 3: &m.W1,
		 4: &m.W2,
		 5: &m.W3,
		 6: &m.A1,
		 7: &m.A2,
		 8: &m.A3,
		 9: &m.B1,
		10: &m.B2,
		11: &m.B3,
		12: &m.M1,
		13: &m.M2,
		14: &m.M3,
	}
}

func TestJacobianMeasurement(t *testing.T) {
	for n:=0; n<100; n++ {
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
				if math.Abs(dM - h.Get(j, i)) > 1e-4 {
					log.Printf("Error in index %2d,%2d: Calc %6f, Jacobian was %6f\n", j, i, dM, h.Get(j, i))
					t.Fail()
				}
			}
		}
	}
}

func TestJacobianState(t *testing.T) {
	//TODO westphae: loop over 100, re-seed
	for n:=0; n<1; n++ {
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
			if i>=6 && i<=9 {
				r := math.Sqrt((1-*(ssmap[i])**(ssmap[i]))/(1-(*(ssmap[i])-Small)*(*(ssmap[i])-Small)))
				for j:= 6; j<=9; j++ {
					if j!=i {
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
				if math.Abs(dS - f.Get(j, i)) > 1e-4 {
					log.Printf("Error in index %2d,%2d: Calc %6f, Jacobian was %6f\n", j, i, dS, f.Get(j, i))
					t.Fail()
				} else if math.Abs(dS) > Small {
					log.Printf("Passed   index %2d,%2d: Calc %6f, Jacobian was %6f\n", j, i, dS, f.Get(j, i))
				}
			}
		}
	}
}
