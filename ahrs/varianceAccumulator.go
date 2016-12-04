package ahrs

import "math"

// NewVarianceAccumulator(init, decay float64) returns a function that,
// when passed a float, accumulates the exponentially weighted mean and
// variance of the first differences with decay constant "decay".
// The accumulator is initialized with a typical mean m0 and variance v0 and
// returns the current estimates of the effective number of observations,
// the mean and the variance.
func NewVarianceAccumulator(m0, v0, decay float64) func(float64) (float64, float64, float64) {
	var (
		l float64 = 0  // last observation
		n float64 = 0  // effective number of observations
		m float64 = m0 // running mean
		v float64 = v0 // running variance
	)

	f := func(obs float64) (float64, float64, float64) {
		if !math.IsNaN(obs) {
			d := (obs - l) - m
			dm := (1 - decay) * d

			l = obs
			n = 1 + decay*(n)
			m += dm
			v = decay * (v + dm*d)
		}
		return n, m, v
	}
	return f
}
