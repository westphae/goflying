package ahrs

import "math"

// ToQuaternion calculates the 0,1,2,3 components of the rotation quaternion
// corresponding to the Tait-Bryan angles phi, theta, psi
func ToQuaternion(phi, theta, psi float64) (float64, float64, float64, float64) {
	phi = -phi            // We want psi positive to mean a roll to the right
	psi = psi - math.Pi/2 // We want psi=0 means north, psi=Pi/2 means east
	cphi := math.Cos(phi / 2)
	sphi := math.Sin(phi / 2)
	ctheta := math.Cos(theta / 2)
	stheta := math.Sin(theta / 2)
	cpsi := math.Cos(psi / 2)
	spsi := math.Sin(psi / 2)

	q0 := cphi*ctheta*cpsi - sphi*stheta*spsi
	q1 := sphi*ctheta*cpsi + cphi*stheta*spsi
	q2 := cphi*stheta*cpsi - sphi*ctheta*spsi
	q3 := cphi*ctheta*spsi + sphi*stheta*cpsi
	return q0, q1, q2, q3
}

// FromQuaternion calculates the Tait-Bryan angles phi, theta, psi corresponding to
// the quaternion
func FromQuaternion(q0, q1, q2, q3 float64) (float64, float64, float64) {
	phi := math.Atan2(-2*(q0*q1-q2*q3), q0*q0-q1*q1-q2*q2+q3*q3)
	theta := math.Asin(2 * (q0*q2 + q3*q1) / math.Sqrt(q0*q0+q1*q1+q2*q2+q3*q3))
	psi := math.Pi/2 + math.Atan2(2*(q0*q3-q1*q2), q0*q0+q1*q1-q2*q2-q3*q3)
	if psi < -1e-4 {
		psi += 2 * math.Pi
	}
	return phi, theta, psi
}

// VarFromQuaternion returns the standard deviation of the Tate-Bryan angles phi, theta, psi
// corresponding to the quaternion q0, q1, q2, q3 with stdev dq0, dq1, dq2, dq3
func VarFromQuaternion(q0, q1, q2, q3, dq0, dq1, dq2, dq3 float64) (float64, float64, float64) {
	var qq, rr, denom float64
	qq = q0*q0 - q1*q1 - q2*q2 + q3*q3
	rr = q0*q1 - q2*q3
	denom = 4*rr*rr + qq*qq
	dphidq0 := (-2*q1*qq + 4*q0*rr)/denom
	dphidq1 := (-2*q0*qq - 4*q1*rr)/denom
	dphidq2 := (+2*q3*qq - 4*q2*rr )/denom
	dphidq3 := (+2*q2*qq + 4*q3*rr)/denom
	rr = 2/math.Sqrt(1-4*(q0*q2 + q1*q3)*(q0*q2 + q1*q3))
	dthetadq0 := q2*rr
	dthetadq1 := q3*rr
	dthetadq2 := q0*rr
	dthetadq3 := q1*rr
	qq = q0*q0 + q1*q1 - q2*q2 - q3*q3
	rr = q0*q3 - q1*q2
	denom = 4*(q0*q3 - q1*q2)*(q0*q3 - q1*q2) + qq*qq
	dpsidq0 := (+2*q3*qq - 4*q0*rr)/denom
	dpsidq1 := (-2*q2*qq - 4*q1*rr)/denom
	dpsidq2 := (-2*q1*qq + 4*q2*rr)/denom
	dpsidq3 := (+2*q0*qq + 4*q3*rr)/denom
	return (dphidq0*dq0 + dphidq1*dq1 +dphidq2*dq2 +dphidq3*dq3),
		(dthetadq0*dq0 + dthetadq1*dq1 +dthetadq2*dq2 +dthetadq3*dq3),
		(dpsidq0*dq0 + dpsidq1*dq1 +dpsidq2*dq2 +dpsidq3*dq3)
}
