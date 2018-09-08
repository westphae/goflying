// The Simple procedure bases K, L on the measured min/max value along each axis.
// It can provide a useful calibration but requires careful manual manipulation.
package magkal

import (
	"math"

	"../ahrs"
)


func ComputeSimple(s MagKalState, cIn chan ahrs.Measurement, cOut chan MagKalState) {
	var (
		m1Min, m1Max = Big, -Big
		m2Min, m2Max = Big, -Big
		m3Min, m3Max = Big, -Big
	)

	for m := range cIn { // Receive input measurements
		s.T = m.T // Update the MagKalState
		m1Min, m1Max = math.Min(m1Min, m.M1), math.Max(m1Max, m.M1)
		m2Min, m2Max = math.Min(m2Min, m.M2), math.Max(m2Max, m.M2)
		m3Min, m3Max = math.Min(m3Min, m.M3), math.Max(m3Max, m.M3)

		if m1Max-m1Min > 2*AvgMagField/s.K[0] {
			s.K[0] = 2 * AvgMagField / (m1Max - m1Min)
			s.L[0] = -s.K[0] * (m1Max + m1Min) / 2
		}

		if m2Max-m2Min > 2*AvgMagField/s.K[1] {
			s.K[1] = 2 * AvgMagField / (m2Max - m2Min)
			s.L[1] = -s.K[1] * (m2Max + m2Min) / 2
		}

		if m3Max-m3Min > 2*AvgMagField/s.K[2] {
			s.K[2] = 2 * AvgMagField / (m3Max - m3Min)
			s.L[2] = -s.K[2] * (m3Max + m3Min) / 2
		}

		s.updateLogMap(&m, s.LogMap)
		select {
		case cOut <- s: // Send results when requested, non-blocking
		default:
		}
	}

	close(cOut) // When cIn is closed, close cOut
}
