package ahrs

import (
	"math"
)

// ToQuaternion calculates the 0,1,2,3 components of the rotation quaternion
// corresponding to the Tait-Bryan angles phi, theta, psi
func ToQuaternion(phi, theta, psi float64) (float64, float64, float64, float64) {
	theta = -theta        // We want positive theta to mean pitch up
	psi = math.Pi/2 - psi // We want psi to go N-E-S-W
	cphi := math.Cos(phi / 2)
	sphi := math.Sin(phi / 2)
	ctheta := math.Cos(theta / 2)
	stheta := math.Sin(theta / 2)
	cpsi := math.Cos(psi / 2)
	spsi := math.Sin(psi / 2)

	q0 := cphi*ctheta*cpsi + sphi*stheta*spsi
	q1 := sphi*ctheta*cpsi - cphi*stheta*spsi
	q2 := cphi*stheta*cpsi + sphi*ctheta*spsi
	q3 := cphi*ctheta*spsi - sphi*stheta*cpsi
	return q0, q1, q2, q3
}

// FromQuaternion calculates the Tait-Bryan angles phi, theta, psi corresponding to
// the quaternion
func FromQuaternion(q0, q1, q2, q3 float64) (phi float64, theta float64, psi float64) {
	phi = math.Atan2(2*(q0*q1+q2*q3), (q0*q0 - q1*q1 - q2*q2 + q3*q3))

	v := -2 * (q0*q2 - q3*q1) / (q0*q0 + q1*q1 + q2*q2 + q3*q3)
	if v >= 1 {
		theta = Pi / 2
	} else if v <= -1 {
		theta = -Pi / 2
	} else {
		theta = math.Asin(v)
	}
	psi = math.Pi/2 - math.Atan2(2*(q0*q3+q1*q2), (q0*q0+q1*q1-q2*q2-q3*q3))
	if psi < 0 {
		psi += 2 * math.Pi
	}
	return
}

// VarFromQuaternion returns the standard deviation of the Tate-Bryan angles phi, theta, psi
// corresponding to the quaternion q0, q1, q2, q3 with stdev dq0, dq1, dq2, dq3
func VarFromQuaternion(q0, q1, q2, q3, dq0, dq1, dq2, dq3 float64) (float64, float64, float64) {
	var qq, rr, denom float64
	rr = 2 * (q0*q1 + q2*q3)
	qq = q0*q0 - q1*q1 - q2*q2 + q3*q3
	denom = rr*rr + qq*qq
	dphidq0 := (2*q1*qq - 2*q0*rr) / denom
	dphidq1 := (2*q0*qq + 2*q1*rr) / denom
	dphidq2 := (2*q3*qq + 2*q2*rr) / denom
	dphidq3 := (2*q2*qq - 2*q3*rr) / denom
	rr = 2 * (q0*q2 - q1*q3)
	qq = q0*q0 + q1*q1 + q2*q2 + q3*q3
	denom = qq * math.Sqrt(qq*qq-rr*rr)
	dthetadq0 := (2*q0*rr - 2*q2*qq) / denom
	dthetadq1 := (2*q1*rr + 2*q3*qq) / denom
	dthetadq2 := (2*q2*rr - 2*q0*qq) / denom
	dthetadq3 := (2*q3*rr + 2*q1*qq) / denom
	rr = 2 * (q0*q3 + q1*q2)
	qq = q0*q0 + q1*q1 - q2*q2 - q3*q3
	denom = 4*(q0*q3-q1*q2)*(q0*q3-q1*q2) + qq*qq
	dpsidq0 := (-2*q3*qq + 2*q0*rr) / denom
	dpsidq1 := (-2*q2*qq + 2*q1*rr) / denom
	dpsidq2 := (-2*q1*qq - 2*q2*rr) / denom
	dpsidq3 := (-2*q0*qq - 2*q3*rr) / denom
	return (dphidq0*dq0 + dphidq1*dq1 + dphidq2*dq2 + dphidq3*dq3),
		(dthetadq0*dq0 + dthetadq1*dq1 + dthetadq2*dq2 + dthetadq3*dq3),
		(dpsidq0*dq0 + dpsidq1*dq1 + dpsidq2*dq2 + dpsidq3*dq3)
}

// QuaternionAToB constructs a quaternion Q such that QAQ* = B for arbitrary vectors A and B
func QuaternionAToB(a1, a2, a3, b1, b2, b3 float64) (q0, q1, q2, q3 float64) {
	aa := math.Sqrt(a1*a1 + a2*a2 + a3*a3)
	bb := math.Sqrt(b1*b1 + b2*b2 + b3*b3)
	ab := a1*b1 + a2*b2 + a3*b3

	q0 = aa*bb + ab
	if q0 < Small { // break the degeneracy
		q1 = a2*(a3+1) - a3*a2
		q2 = a3*a1 - a1*(a3+1)
		q3 = 0
	} else {
		q1 = a2*b3 - a3*b2
		q2 = a3*b1 - a1*b3
		q3 = a1*b2 - a2*b1
	}

	return QuaternionNormalize(q0, q1, q2, q3)
}

// QuaternionRotate rotates a quaternion Qea by a small rate-of-change vector Ha
// (e.g. as measured by a gyro in the aircraft frame), Qea -> Qea + 0.5*Qea*Ha
func QuaternionRotate(q0, q1, q2, q3, h1, h2, h3 float64) (r0, r1, r2, r3 float64) {
	r0 = q0 + 0.5*(-q1*h1-q2*h2-q3*h3)
	r1 = q1 + 0.5*(q0*h1-q3*h2+q2*h3)
	r2 = q2 + 0.5*(q3*h1+q0*h2-q1*h3)
	r3 = q3 + 0.5*(-q2*h1+q1*h2+q0*h3)
	return QuaternionNormalize(r0, r1, r2, r3)
}

// RotationMatrixToQuaternion computes the quaternion q corresponding to a rotation matrix r.
func RotationMatrixToQuaternion(r [3][3]float64) (q0, q1, q2, q3 float64) {
	q0 = math.Sqrt(1 + r[0][0] + r[1][1] + r[2][2])/2
	q1 = (r[2][1] - r[1][2])/(4*q0)
	q2 = (r[0][2] - r[2][0])/(4*q0)
	q3 = (r[1][0] - r[0][1])/(4*q0)
	return
}

// QuaternionToRotationMatrix computes the rotation matrix r corresponding to a quaternion q.
func QuaternionToRotationMatrix(q0, q1, q2, q3 float64) (r *[3][3]float64) {
	r = new([3][3]float64)
	r[0][0] = +q0*q0 + q1*q1 - q2*q2 - q3*q3
	r[0][1] = 2 * (-q0*q3 + q1*q2)
	r[0][2] = 2 * (+q0*q2 + q1*q3)
	r[1][0] = 2 * (+q0*q3 + q2*q1)
	r[1][1] = +q0*q0 - q1*q1 + q2*q2 - q3*q3
	r[1][2] = 2 * (-q0*q1 + q2*q3)
	r[2][0] = 2 * (-q0*q2 + q3*q1)
	r[2][1] = 2 * (+q0*q1 + q3*q2)
	r[2][2] = +q0*q0 - q1*q1 - q2*q2 + q3*q3
	return
}

// QuaternionNormalize re-scales the input quaternion to unit norm.
func QuaternionNormalize(q0, q1, q2, q3 float64) (r0, r1, r2, r3 float64) {
	qq := math.Sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
	r0 = q0 / qq
	r1 = q1 / qq
	r2 = q2 / qq
	r3 = q3 / qq
	return
}

// QuaternionSign chooses the sign of quaternion q so that it is minimally distant from quaternion r.
func QuaternionSign(q0, q1, q2, q3, r0, r1, r2, r3 float64) (s0, s1, s2, s3 float64) {
	ss := (q0-r0)*(q0-r0) + (q1-r1)*(q1-r1) + (q2-r2)*(q2-r2) + (q3-r3)*(q3-r3)
	if ss > 2 {
		return -q0, -q1, -q2, -q3
	} else {
		return q0, q1, q2, q3
	}
}
