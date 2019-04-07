// The Trivial procedure is just maintaining constant K, L.
// This is mainly used for testing the integration with Stratux.
package magkal

import "../ahrs"

func ComputeTrivial(s MagKalState, cIn chan ahrs.Measurement, cOut chan MagKalState) {
	if NormVec(s.K) < Small {
		s.K = [3]float64{1, 1, 1}
		s.L = [3]float64{0, 0, 0}
	}

	for m := range cIn { // Receive input measurements
		s.T = m.T // Update the MagKalState
		s.updateLogMap(&m, s.LogMap)
		cOut <- s // Send results when requested, blocking
	}

	close(cOut) // When cIn is closed, close cOut
}
