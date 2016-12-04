The goflying/ahrs package implements AHRS using one of several methods.

### Simple
Uses GPS for primary attitude estimation, assuming coordinated turns and no mushing;
then corrects this estimation using sensor values in a simple/silly way.

### Heuristic:

### Kalman
The rest are different stages of Kalman filter, using various inputs to validate at each stage.

#### Stage 1: gyro only
To begin, we use only the gyro measurements--we ignore all other measurements.
We are going to estimate only the orientation quaternion E and the sensor drifts D.
As a test, we'll also include the calibration oriention quaternion F in the state,
but it should not change.

Since we have no further data to fuse, we don't expect our uncertainties to improve--they
should only get worse as uncertainty is gathered.  However, because we measure the offsets D,
the attitude should not drift much (the effect of D should be to subtract the long-term mean).

Result: Indeed, this is what we see.  There is a very slow drift, but it never gets very big.
The attitude estimation tracks fairly well with what we experience, though the uncertainties
remain large and growing (aside from the uncertainty in the estimated gyro rates H).

We will need a further source of data in order for uncertainties to improve from their initial values.

#### Stage 2: gyro + accel


#### Stage 3: gyro + accel + magnetometer
#### Stage 4: gyro + accel + magnetometer + GPS/baro
#### Stage 5: gyro + accel + magnetometer + GPS/baro + airspeed
