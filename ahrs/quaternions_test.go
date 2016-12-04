package ahrs

import (
	"fmt"
	"math"
	"testing"

	"github.com/westphae/quaternion"
	"math/rand"
)

var (
	c30 = math.Sqrt(3) / 2
	c60 = 0.5
)

const Tolerance = 1e-4

// NotSmall checks whether a result is not small compared to the const Small
func notSmall(x float64) bool {
	return math.Abs(x) > Tolerance
}

// checkQ checks the quaternion against specific roll, pitch and yaw values
func checkQ(q quaternion.Quaternion, r, p, y float64) bool {
	qr, qp, qy := FromQuaternion(q.W, q.X, q.Y, q.Z)
	dqr := qr - r
	if dqr < -Pi {
		dqr += 2 * Pi
	}
	dqp := qp - p
	if dqp < -Pi {
		dqp += 2 * Pi
	}
	dqy := qy - y
	if dqy > Pi {
		dqy -= 2 * Pi
	}
	if dqy < -Pi {
		dqy += 2 * Pi
	}
	wrong := (notSmall(dqr) || notSmall(dqp) || notSmall(dqy))
	if wrong {
		fmt.Printf("Roll: %1.4f==%1.4f, Pitch %1.4f==%1.4f, Yaw %1.4f==%1.4f\n", qr, r, qp, p, qy, y)
	}
	return !wrong
}

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
		if notSmall(phi-phiOut) || notSmall(theta-thetaOut) || notSmall(psi-psiOut) {
			fmt.Println("Testing Qae")
			fmt.Printf("%+5.3f -> %+5.3f, %+5.3f -> %+5.3f, %+5.3f -> %+5.3f\n",
				phi, phiOut, theta, thetaOut, psi, psiOut)
			t.Fail()
		}

	}
}

func TestSpecificToQuaternion(t *testing.T) {
	phis :=   []float64{ 0,    0,  0,      0,      0,    0,        0,  Pi/3, Pi/3, -2*Pi/3}
	thetas := []float64{ 0,    0,  0,      0,      0, Pi/3,    -Pi/3,     0,    0,       0}
	psis :=   []float64{ 0, Pi/2, Pi, 4*Pi/3, 3*Pi/2, Pi/2,   5*Pi/3,  Pi/2,    0,      Pi}
	u1s :=    []float64{ 0,    1,  0,   -c30,     -1,  c60, -c30*c60,     1,    0,       0}
	u2s :=    []float64{ 1,    0, -1,   -c60,      0,    0,  c60*c60,     0,    1,      -1}
	u3s :=    []float64{ 0,    0,  0,      0,      0,  c30,     -c30,     0,    0,       0}
	v1s :=    []float64{-1,    0,  1,    c60,      0,    0,     -c60,     0, -c60,    -c60}
	v2s :=    []float64{ 0,    1,  0,   -c30,     -1,    1,     -c30,   c60,    0,       0}
	v3s :=    []float64{ 0,    0,  0,      0,      0,    0,        0,   c30,  c30,    -c30}
	w1s :=    []float64{ 0,    0,  0,      0,      0, -c30, -c30*c30,     0,  c30,     c30}
	w2s :=    []float64{ 0,    0,  0,      0,      0,    0,  c60*c30,  -c30,    0,       0}
	w3s :=    []float64{ 1,    1,  1,      1,      1,  c60,      c60,   c60,  c60,    -c60}

	x := quaternion.Quaternion{X: 1}
	y := quaternion.Quaternion{Y: 1}
	z := quaternion.Quaternion{Z: 1}
	var (
		e0, e1, e2, e3         float64
		u, v, w, e, uu, vv, ww quaternion.Quaternion
	)

	for i := 0; i < len(phis); i++ {
		u = quaternion.Quaternion{X: u1s[i], Y: u2s[i], Z: u3s[i]}
		v = quaternion.Quaternion{X: v1s[i], Y: v2s[i], Z: v3s[i]}
		w = quaternion.Quaternion{X: w1s[i], Y: w2s[i], Z: w3s[i]}
		e0, e1, e2, e3 = ToQuaternion(phis[i], thetas[i], psis[i])
		e = quaternion.Quaternion{W: e0, X: e1, Y: e2, Z: e3}
		uu = quaternion.Prod(e, x, quaternion.Conj(e))
		vv = quaternion.Prod(e, y, quaternion.Conj(e))
		ww = quaternion.Prod(e, z, quaternion.Conj(e))

		if notSmall(u.W-uu.W) || notSmall(u.X-uu.X) ||
			notSmall(u.Y-uu.Y) || notSmall(u.Z-uu.Z) {
			fmt.Println("Testing specific quaternions")
			fmt.Printf("%d u: %+5.3f -> %+5.3f, %+5.3f -> %+5.3f, %+5.3f -> %+5.3f, %+5.3f -> %+5.3f\n",
				i, u.W, uu.W, u.X, uu.X, u.Y, uu.Y, u.Z, uu.Z)
			t.Fail()
		}

		if notSmall(v.W-vv.W) || notSmall(v.X-vv.X) ||
			notSmall(v.Y-vv.Y) || notSmall(v.Z-vv.Z) {
			fmt.Println("Testing specific quaternions")
			fmt.Printf("%d v: %+5.3f -> %+5.3f, %+5.3f -> %+5.3f, %+5.3f -> %+5.3f, %+5.3f -> %+5.3f\n",
				i, v.W, vv.W, v.X, vv.X, v.Y, vv.Y, v.Z, vv.Z)
			t.Fail()
		}

		if notSmall(w.W-ww.W) || notSmall(w.X-ww.X) ||
			notSmall(w.Y-ww.Y) || notSmall(w.Z-ww.Z) {
			fmt.Println("Testing specific quaternions")
			fmt.Printf("%d w: %+5.3f -> %+5.3f, %+5.3f -> %+5.3f, %+5.3f -> %+5.3f, %+5.3f -> %+5.3f\n",
				i, w.W, ww.W, w.X, ww.X, w.Y, ww.Y, w.Z, ww.Z)
			t.Fail()
		}
	}
}

// A negative pitch rate about the y-axis should pitch the nose up
func TestPitchRotationQuaternion(t *testing.T) {

	q_nose_aircraft := quaternion.Quaternion{0, 1, 0, 0}
	q_rt_wing_aircraft := quaternion.Quaternion{0, 0, -1, 0}
	q_ae := quaternion.Quaternion{1, 0, 0, 0} // headed East
	h_a := quaternion.Quaternion{1, 0, -0.5 * Pi / 180, 0}

	q_nose_pitched_a := quaternion.Prod(h_a, q_nose_aircraft, quaternion.Conj(h_a))
	q_nose_pitched_e := quaternion.Prod(quaternion.Conj(q_ae), q_nose_pitched_a, q_ae)
	q_rt_wing_pitched_a := quaternion.Prod(h_a, q_rt_wing_aircraft, quaternion.Conj(h_a))
	q_rt_wing_pitched_e := quaternion.Prod(quaternion.Conj(q_ae), q_rt_wing_pitched_a, q_ae)
	if q_nose_pitched_e.Z < Tolerance || notSmall(q_nose_pitched_e.Y) ||
		notSmall(q_rt_wing_pitched_e.X) || notSmall(q_rt_wing_pitched_e.Z) {
		fmt.Println("Testing pitch directionality")
		fmt.Println(q_nose_pitched_e)
		fmt.Println(q_rt_wing_pitched_e)
		t.Fail()
	}
}

// A positive roll rate about the x-axis should roll the right wing down
func TestRollRotationQuaternion(t *testing.T) {

	q_nose_aircraft := quaternion.Quaternion{0, 1, 0, 0}
	q_rt_wing_aircraft := quaternion.Quaternion{0, 0, -1, 0}
	q_ae := quaternion.Quaternion{1, 0, 0, 0} // headed East
	h_a := quaternion.Quaternion{1, 0.5 * Pi / 180, 0, 0}

	q_nose_rolled_a := quaternion.Prod(h_a, q_nose_aircraft, quaternion.Conj(h_a))
	q_nose_rolled_e := quaternion.Prod(quaternion.Conj(q_ae), q_nose_rolled_a, q_ae)
	q_rt_wing_rolled_a := quaternion.Prod(h_a, q_rt_wing_aircraft, quaternion.Conj(h_a))
	q_rt_wing_rolled_e := quaternion.Prod(quaternion.Conj(q_ae), q_rt_wing_rolled_a, q_ae)
	if notSmall(q_nose_rolled_e.Z) || notSmall(q_nose_rolled_e.Y) ||
		q_rt_wing_rolled_e.Z > Tolerance || notSmall(q_rt_wing_rolled_e.X) {
		fmt.Println("Testing roll directionality")
		fmt.Println(q_nose_rolled_e)
		fmt.Println(q_rt_wing_rolled_e)
		t.Fail()
	}
}

// A negative yaw rate about the z-axis should turn the nose to the right
func TestYawRotationQuaternion(t *testing.T) {

	q_nose_aircraft := quaternion.Quaternion{0, 1, 0, 0}
	q_rt_wing_aircraft := quaternion.Quaternion{0, 0, -1, 0}
	q_ae := quaternion.Quaternion{1, 0, 0, 0} // headed East
	h_a := quaternion.Quaternion{1, 0, 0, -0.5 * Pi / 180}

	q_nose_yawed_a := quaternion.Prod(h_a, q_nose_aircraft, quaternion.Conj(h_a))
	q_nose_yawed_e := quaternion.Prod(quaternion.Conj(q_ae), q_nose_yawed_a, q_ae)
	q_rt_wing_yawed_a := quaternion.Prod(h_a, q_rt_wing_aircraft, quaternion.Conj(h_a))
	q_rt_wing_yawed_e := quaternion.Prod(quaternion.Conj(q_ae), q_rt_wing_yawed_a, q_ae)
	if notSmall(q_nose_yawed_e.Z) || q_nose_yawed_e.X < Tolerance ||
		notSmall(q_rt_wing_yawed_e.Z) || q_rt_wing_yawed_e.X > -Tolerance {
		fmt.Println("Testing yaw directionality")
		fmt.Println(q_nose_yawed_e)
		fmt.Println(q_rt_wing_yawed_e)
		t.Fail()
	}
}

// Quickie half-angle formulae for quaternions works
func TestQuaternionHalfAngles(t *testing.T) {

	psis := []float64{0, Pi / 2, Pi, 4 * Pi / 3, 3 * Pi / 2, Pi / 2, 5 * Pi / 3, Pi / 2, 0, Pi}

	var (
		w1, w2, u, psi, e0, e3 float64
	)

	for i := 0; i < len(psis); i++ {
		w1, w2 = math.Sin(psis[i]), math.Cos(psis[i])
		u = math.Hypot(w1, w2)
		e0, e3 = math.Sqrt((u+w1)/(2*u)), math.Sqrt((u-w1)/(2*u))
		if w2 < 0 {
			e3 *= -1
		}
		_, _, psi = FromQuaternion(e0, 0, 0, e3)

		if notSmall(psi - psis[i]) {
			fmt.Println("Testing Quaternion quickie half-angle formula")
			t.Fail()
		}
	}
}

// Rotating vector A into B works
func TestQuaternionAToB(t *testing.T) {
	var (
		a1, a2, a3, aa float64
		b1, b2, b3, bb float64
		q0, q1, q2, q3 float64
		a, q, z        quaternion.Quaternion
	)
	for i := 0; i < 100; i++ {
		a1 = 2*rand.Float64() - 1
		a2 = 2*rand.Float64() - 1
		a3 = 2*rand.Float64() - 1
		aa = math.Sqrt(a1*a1 + a2*a2 + a3*a3)

		b1 = 2*rand.Float64() - 1
		b2 = 2*rand.Float64() - 1
		b3 = 2*rand.Float64() - 1
		bb = math.Sqrt(b1*b1 + b2*b2 + b3*b3)

		q0, q1, q2, q3 = QuaternionAToB(a1, a2, a3, b1, b2, b3)
		a = quaternion.Quaternion{0, a1 / aa, a2 / aa, a3 / aa}
		q = quaternion.Quaternion{q0, q1, q2, q3}
		z = quaternion.Prod(q, a, quaternion.Conj(q))
		if notSmall(z.W) || notSmall(z.X-b1/bb) ||
			notSmall(z.Y-b2/bb) || notSmall(z.Z-b3/bb) {
			fmt.Printf("A:  %4f %4f %4f\n", a1, a2, a3)
			fmt.Printf("B:  %4f %4f %4f\n", b1, b2, b3)
			fmt.Printf("Got %4f %4f %4f (%4f)\n", z.X*bb/aa, z.Y*bb/aa, z.Z*bb/aa, z.W*bb/aa)
			t.Fail()
		}
	}

	// Additional test: opposite vectors
	q0, q1, q2, q3 = QuaternionAToB(a1, a2, a3, -a1, -a2, -a3)
	a = quaternion.Quaternion{0, a1, a2, a3}
	q = quaternion.Quaternion{q0, q1, q2, q3}
	z = quaternion.Prod(q, a, quaternion.Conj(q))
	if notSmall(z.W) || notSmall(z.X+a1) ||
		notSmall(z.Y+a2) || notSmall(z.Z+a3) {
		fmt.Printf("A:  %4f %4f %4f\n", a1, a2, a3)
		fmt.Printf("B:  %4f %4f %4f\n", -a1, -a2, -a3)
		fmt.Printf("Got %4f %4f %4f (%4f)\n", z.X, z.Y, z.Z, z.W)
		fmt.Printf("Q:  %4f %4f %4f (%4f)\n", q.X, q.Y, q.Z, q.W)
		t.Fail()
	}
}

// Composing pitch & yaw rotations results in just separate net rotations
func TestMultipleRotations(t *testing.T) {
	// Starting orientation: nose pointing East, no roll
	q0 := quaternion.Quaternion{1, 0, 0, 0}
	q0 = quaternion.Unit(q0)
	p := Pi / 3  // Pitch up
	r := Pi / 4  // Roll right
	y := +Pi / 2 // Yaw left

	// Define some rotations in appropriate frame
	qap1 := quaternion.Quaternion{math.Cos(-p / 2), 0, math.Sin(-p / 2), 0}
	qap2 := quaternion.Conj(qap1)                                         // Conj for opposite action
	qey := quaternion.Quaternion{math.Cos(y / 2), 0, 0, math.Sin(y / 2)}  // Yaw is always defined in the earth frame, no matter the attitude
	qar1 := quaternion.Quaternion{math.Cos(r / 2), math.Sin(r / 2), 0, 0} // Roll is always defined in the aircraft frame, no matter the attitude
	qar2 := quaternion.Conj(qar1)

	// Earth frame
	qe := qap1
	if !checkQ(qe, 0, p, Pi/2) {
		t.Fail()
	}
	qe = quaternion.Prod(quaternion.Prod(qe, qar1, quaternion.Conj(qe)), qe) // How we translate airplane frame to earth frame
	if !checkQ(qe, r, p, Pi/2) {
		t.Fail()
	}
	qe = quaternion.Prod(qey, qe)
	if !checkQ(qe, r, p, Pi/2-y) {
		t.Fail()
	}
	qe = quaternion.Prod(quaternion.Prod(qe, qar2, quaternion.Conj(qe)), qe)
	if !checkQ(qe, 0, p, Pi/2-y) {
		t.Fail()
	}
	qe = quaternion.Prod(quaternion.Prod(qe, qap2, quaternion.Conj(qe)), qe)
	if !checkQ(qe, 0, 0, Pi/2-y) {
		t.Fail()
	}
	qe = quaternion.Prod(qe, q0)

	// Airplane frame
	qa := qap1
	if !checkQ(qa, 0, p, Pi/2) {
		t.Fail()
	}
	qa = quaternion.Prod(qa, qar1)
	if !checkQ(qa, r, p, Pi/2) {
		t.Fail()
	}
	qa = quaternion.Prod(qa, quaternion.Prod(quaternion.Conj(qa), qey, qa)) // How we translate earth frame to airplane frame
	if !checkQ(qa, r, p, Pi/2-y) {
		t.Fail()
	}
	qa = quaternion.Prod(qa, qar2)
	if !checkQ(qa, 0, p, Pi/2-y) {
		t.Fail()
	}
	qa = quaternion.Prod(qa, qap2)
	if !checkQ(qa, 0, 0, Pi/2-y) {
		t.Fail()
	}
	qa = quaternion.Prod(qa, q0)
}

// Composing a large number of small aircraft-frame rotations (e.g. from a sensor) adds up to the net earth-frame rotation
func TestSmallCompositions(t *testing.T) {
	// Starting orientation: nose pointing East, no roll
	q0 := quaternion.Quaternion{1, 0, 0, 0}
	n := 100                    // Number of divisions for each rotation
	dp := (Pi / 3) / float64(n) // Pitch up 60°
	dr := (Pi / 4) / float64(n) // Roll right 45°
	dy := (Pi / 2) / float64(n) // Yaw left 90°

	// Define some rotations in earth frame
	var qqs []quaternion.Quaternion = []quaternion.Quaternion{
		quaternion.Quaternion{math.Cos(-dp / 2), 0, math.Sin(-dp / 2), 0}, // Pitch up
		quaternion.Quaternion{math.Cos(-dr / 2), math.Sin(-dr / 2), 0, 0}, // Roll left (aircraft frame!)
		quaternion.Quaternion{math.Cos(dy / 2), 0, 0, math.Sin(dy / 2)},   // Yaw left
		quaternion.Quaternion{math.Cos(dr / 2), math.Sin(dr / 2), 0, 0},   // Roll right (aircraft frame!)
		quaternion.Quaternion{math.Cos(-dp / 2), math.Sin(-dp / 2), 0, 0}, // Pitch down
	}

	// Apply the rotations successively
	var qqa quaternion.Quaternion
	qa := q0
	qe := q0
	for _, qq := range qqs {
		for i := 0; i < n; i++ {
			// Convert aircraft frame to earth frame
			qqs[1] = quaternion.Prod(qq, qqs[1], quaternion.Conj(qq))
			qqs[3] = quaternion.Prod(qq, qqs[3], quaternion.Conj(qq))

			// Apply to current earth-frame orientation
			qe = quaternion.Prod(qq, qe)

			// Apply to current aircraft-frame orientation
			qqa = quaternion.Prod(quaternion.Conj(qa), qq, qa) // Translate from earth to aircraft frame
			qa0, qa1, qa2, qa3 := QuaternionRotate(qa.W, qa.X, qa.Y, qa.Z, 2*qqa.X, 2*qqa.Y, 2*qqa.Z)
			qa = quaternion.Quaternion{qa0, qa1, qa2, qa3}
		}
	}

	// Result should be just a rotation from East to North
	fmt.Println("Checking aircraft frame result:")
	if !checkQ(qa, 0, 0, 0) {
		t.Fail()
	}
	fmt.Println("Checking earth frame result:")
	if !checkQ(qe, 0, 0, 0) {
		t.Fail()
	}
}
