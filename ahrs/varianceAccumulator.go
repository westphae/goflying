package ahrs

// NewVarianceAccumulator(init, decay float64) returns a function that,
// when passed a float, accumulates an exponentially weighted mean and
// variance with decay constant "decay".  The accumulator is initialized
// with an observation "init" and returns the current estimates of the
// effective number of observations, the mean and the variance.
func NewVarianceAccumulator(init, decay float64) (func(float64) (float64, float64, float64)) {
	var (
		n float64 = 1
		m float64 = init
		v float64 = 0
	)

	f := func(obs float64) (float64, float64, float64) {
		d := obs - m
		dm := (1-decay) * d

		n = 1 + decay*(n)
		m += dm
		v = decay*(v + dm * d)
		return n, m, v
	}
	return f
}