package ahrs

import (
	"fmt"
	"math"
	"testing"

	"github.com/westphae/quaternion"
)

const (
	PI = math.Pi
	SMALL = 1e-6
)

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
		if math.Abs(phi-phiOut) > SMALL || math.Abs(theta-thetaOut) > SMALL || math.Abs(psi-psiOut) > SMALL {
			fmt.Printf("%+5.3f -> %+5.3f, %+5.3f -> %+5.3f, %+5.3f -> %+5.3f\n",
				phi, phiOut, theta, thetaOut, psi, psiOut)
			t.Fail()
		}

	}
}

func TestSpecificToQuaternion(t *testing.T) {
	var (
		c30 = math.Sqrt(3) / 2
		c60 = 0.5
	)
	phis :=   []float64{ 0,    0,  0,     0,       0,    0,        0,  PI /3, PI /3, -2* PI /3}
	thetas := []float64{ 0,    0,  0,     0,       0, PI /3,    -PI /3,     0,    0,       0}
	psis :=   []float64{ 0, PI /2, PI, 4* PI /3, 3* PI /2, PI /2,   5* PI /3, PI /2,    0, PI}
	u1s :=    []float64{ 0,    1,  0,   -c30,     -1,  c60, -c30*c60,     1,    0,       0}
	u2s :=    []float64{ 1,    0, -1,   -c60,      0,    0,  c60*c60,     0,    1,      -1}
	u3s :=    []float64{ 0,    0,  0,      0,      0,  c30,     -c30,     0,    0,       0}
	v1s :=    []float64{-1,    0,  1,    c60,      0,    0,     -c60,     0, -c60,    -c60}
	v2s :=    []float64{ 0,    1,  0,   -c30,     -1,    1,     -c30,   c60,    0,       0}
	v3s :=    []float64{ 0,    0,  0,      0,      0,    0,        0,   c30,  c30,    -c30}

	x := quaternion.Quaternion{X: 1}
	y := quaternion.Quaternion{Y: 1}
	var (
		e0, e1, e2, e3  float64
		u, v, e, uu, vv quaternion.Quaternion
	)

	for i := 0; i < len(phis); i++ {
		u = quaternion.Quaternion{X: u1s[i], Y: u2s[i], Z: u3s[i]}
		v = quaternion.Quaternion{X: v1s[i], Y: v2s[i], Z: v3s[i]}
		e0, e1, e2, e3 = ToQuaternion(phis[i], thetas[i], psis[i])
		e = quaternion.Quaternion{W: e0, X: e1, Y: e2, Z: e3}
		uu = quaternion.Prod(quaternion.Conj(e), x, e)
		vv = quaternion.Prod(quaternion.Conj(e), y, e)

		if math.Abs(u.W-uu.W) > SMALL || math.Abs(u.X-uu.X) > SMALL ||
			math.Abs(u.Y-uu.Y) > SMALL || math.Abs(u.Z-uu.Z) > SMALL {
			fmt.Println(i)
			fmt.Println(e)
			fmt.Printf("u: %+5.3f -> %+5.3f, %+5.3f -> %+5.3f, %+5.3f -> %+5.3f, %+5.3f -> %+5.3f\n",
				u.W, uu.W, u.X, uu.X, u.Y, uu.Y, u.Z, uu.Z)
			t.Fail()
		}

		if math.Abs(v.W-vv.W) > SMALL || math.Abs(v.X-vv.X) > SMALL ||
			math.Abs(v.Y-vv.Y) > SMALL || math.Abs(v.Z-vv.Z) > SMALL {
			fmt.Println(i)
			fmt.Println(e)
			fmt.Printf("v: %+5.3f -> %+5.3f, %+5.3f -> %+5.3f, %+5.3f -> %+5.3f, %+5.3f -> %+5.3f\n",
				v.W, vv.W, v.X, vv.X, v.Y, vv.Y, v.Z, vv.Z)
			t.Fail()
		}

	}
}

// A negative pitch rate about the y-axis should pitch the nose up
func TestPitchRotationQuaternion(t *testing.T) {

	q_nose_aircraft := quaternion.Quaternion{0, 1, 0, 0}
	q_rt_wing_aircraft := quaternion.Quaternion{0, 0, -1, 0}
	q_ae := quaternion.Quaternion{1, 0, 0, 0} // headed north
	h_a := quaternion.Quaternion{1, 0, -0.5* PI /180, 0}

	q_nose_pitched_a := quaternion.Prod(h_a, q_nose_aircraft, quaternion.Conj(h_a))
	q_nose_pitched_e := quaternion.Prod(quaternion.Conj(q_ae), q_nose_pitched_a, q_ae)
	q_rt_wing_pitched_a := quaternion.Prod(h_a, q_rt_wing_aircraft, quaternion.Conj(h_a))
	q_rt_wing_pitched_e := quaternion.Prod(quaternion.Conj(q_ae), q_rt_wing_pitched_a, q_ae)
	if q_nose_pitched_e.Z < SMALL || math.Abs(q_nose_pitched_e.Y) > SMALL ||
			math.Abs(q_rt_wing_pitched_e.X) > SMALL || math.Abs(q_rt_wing_pitched_e.Z) > SMALL {
		fmt.Println(q_nose_pitched_e)
		fmt.Println(q_rt_wing_pitched_e)
		t.Fail()
	}
}

// A positive roll rate about the x-axis should roll the right wing down
func TestRollRotationQuaternion(t *testing.T) {

	q_nose_aircraft := quaternion.Quaternion{0, 1, 0, 0}
	q_rt_wing_aircraft := quaternion.Quaternion{0, 0, -1, 0}
	q_ae := quaternion.Quaternion{1, 0, 0, 0} // headed north
	h_a := quaternion.Quaternion{1, 0.5* PI /180, 0, 0}

	q_nose_rolled_a := quaternion.Prod(h_a, q_nose_aircraft, quaternion.Conj(h_a))
	q_nose_rolled_e := quaternion.Prod(quaternion.Conj(q_ae), q_nose_rolled_a, q_ae)
	q_rt_wing_rolled_a := quaternion.Prod(h_a, q_rt_wing_aircraft, quaternion.Conj(h_a))
	q_rt_wing_rolled_e := quaternion.Prod(quaternion.Conj(q_ae), q_rt_wing_rolled_a, q_ae)
	if math.Abs(q_nose_rolled_e.Z) > SMALL || math.Abs(q_nose_rolled_e.Y) > SMALL ||
			q_rt_wing_rolled_e.Z > SMALL || math.Abs(q_rt_wing_rolled_e.X) > SMALL {
		fmt.Println(q_nose_rolled_e)
		fmt.Println(q_rt_wing_rolled_e)
		t.Fail()
	}
}

// A negative yaw rate about the z-axis should turn the nose to the right
func TestYawRotationQuaternion(t *testing.T) {

	q_nose_aircraft := quaternion.Quaternion{0, 1, 0, 0}
	q_rt_wing_aircraft := quaternion.Quaternion{0, 0, -1, 0}
	q_ae := quaternion.Quaternion{1, 0, 0, 0} // headed north
	h_a := quaternion.Quaternion{1, 0, 0, -0.5* PI /180}

	q_nose_yawed_a := quaternion.Prod(h_a, q_nose_aircraft, quaternion.Conj(h_a))
	q_nose_yawed_e := quaternion.Prod(quaternion.Conj(q_ae), q_nose_yawed_a, q_ae)
	q_rt_wing_yawed_a := quaternion.Prod(h_a, q_rt_wing_aircraft, quaternion.Conj(h_a))
	q_rt_wing_yawed_e := quaternion.Prod(quaternion.Conj(q_ae), q_rt_wing_yawed_a, q_ae)
	if math.Abs(q_nose_yawed_e.Z) > SMALL || q_nose_yawed_e.X < SMALL ||
			math.Abs(q_rt_wing_yawed_e.Z) > SMALL || q_rt_wing_yawed_e.X > -SMALL {
		fmt.Println(q_nose_yawed_e)
		fmt.Println(q_rt_wing_yawed_e)
		t.Fail()
	}
}
