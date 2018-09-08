// The Trivial procedure is just maintaining constant K, L.
// This is mainly used for testing the integration with Stratux.
package magkal

import "../ahrs"

func ComputeTrivial(s MagKalState, cIn chan ahrs.Measurement, cOut chan MagKalState) {
	for m := range cIn { // Receive input measurements
		s.T = m.T // Update the MagKalState
		s.updateLogMap(&m, s.LogMap)
		select {
		case cOut <- s: // Send results when requested, non-blocking
		default:
		}
	}

	close(cOut) // When cIn is closed, close cOut
}
