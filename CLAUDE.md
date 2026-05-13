# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & test

This is a Go modules project (`go.mod` at the repo root, `go 1.22` — the floor imposed by the `westphae/go-iio` dep; otherwise only Go 1.18 features are used).

External deps (`gorilla/websocket`, `kidoman/embd`, `skelterjohn/go.matrix`, `westphae/go-iio`, `westphae/quaternion`) are pinned in `go.mod` / `go.sum`; `go mod tidy` keeps them in sync. The repo conventionally lives at `$GOPATH/src/github.com/westphae/goflying` because that's where its sibling consumers (see below) expect it, but module mode no longer requires it.

A `replace` directive points `github.com/kidoman/embd` at `github.com/westphae/embd` because upstream embd (last touched 2017) panics on modern Raspberry Pi OS kernel strings like `6.12.62+rpt-rpi-v8` — its `parseVersion` can't handle the `+rpt` suffix on the patch component. The fork has a single-function patch in `detect.go`. The BMP280 driver no longer uses embd (see below); the remaining embd consumers are the ICM-20948 and MPU-9250 drivers. The long-term fix is the periph.io migration listed under deferred modernizations.

Common commands (run from repo root):

- `go test ./ahrs/...` — unit tests (quaternion math, AHRS algorithms). Single test: `go test ./ahrs -run TestName -v`.
- `go build ./sim` — builds the simulation/replay binary (`ahrs_sim`); main package.
- `go build ./ahrsweb/cmd/ahrsweb_server` — websocket visualization server (serves templates from `res/` — run from the cmd dir).
- `go build ./sensors/bmp280/test`, `./sensors/mpu9250/test`, `./sensors/icm20948/test` — small `main` programs that exercise the sensor drivers; these only run on a Raspberry Pi.
- `go build ./...` — compile every package; CI sanity check.

The sensor `test/` binaries will compile on any platform but only function with real hardware. The BMP280 test reads `/sys/bus/iio/devices/...` (kernel driver, see hardware setup below); the MPU-9250 and ICM-20948 tests open `/dev/i2c-1` directly via embd.

### Consumers

Two known downstream consumers live alongside this repo in GOPATH:

- **`../magkal`** — Go-modules build with its own `go.mod` and a `replace github.com/westphae/goflying => ../goflying` directive (plus a matching `replace github.com/kidoman/embd => github.com/westphae/embd ...`, since `replace` in a dep is ignored by Go modules — the main module must restate it). Imports `github.com/westphae/goflying/sensors/icm20948` (and may grow others). Local changes here flow into magkal builds immediately; verify with `cd ../magkal && go build ./...` after touching this repo.
- **`../stratux`** — GOPATH-style; consumes this repo as a **git submodule** mounted at `stratux/goflying/`. The submodule is pinned to a specific commit, so canonical-goflying changes do not reach stratux until its submodule pointer is bumped. Stratux's imports look like `"../goflying/ahrs"` etc.; do not be alarmed by their pre-`3efece4` package layout — they refer to the pinned submodule.

## Architecture

The repo implements an aircraft Attitude and Heading Reference System (AHRS) for the [Stratux](https://github.com/cyoung/stratux) ADS-B receiver, plus the sensor drivers and tooling around it.

### Coordinate frames (load-bearing — used everywhere)

- **Aircraft frame** (non-inertial): axis 1 = nose, axis 2 = left wing, axis 3 = up.
- **Earth frame** (inertial): axis 1 = east, axis 2 = north, axis 3 = up.
- **Sensor frame**: fixed within the aircraft frame but rotated (the rotation is bias quaternion `F`, which the filter estimates).

When reading/writing vector code, confirm which frame the values are in — comments on `State` and `Measurement` fields (`ahrs/ahrs_state.go`, `ahrs/ahrs_defs.go`) are the source of truth.

### `ahrs/` — the filter

`AHRSProvider` (`ahrs_defs.go`) is the public interface every algorithm implements: `Compute(*Measurement)`, `RollPitchHeading()`, `MagHeading()`, `SlipSkid()`, `RateOfTurn()`, `GLoad()`, plus calibration/config/reset. Callers depend only on this interface — they swap algorithms freely.

Implementations:
- `ahrs_simple.go` — GPS-primary heuristic. Assumes coordinated turns; corrects with sensor data. Used as a fallback / easy-to-debug baseline.
- `ahrs_kalman.go`, `ahrs_kalman0.go`, `ahrs_kalman1.go` — Extended Kalman filter variants at successive stages (gyro-only → +accel → +mag → +GPS/baro → +airspeed). See `ahrs/docs/KalmanNotes.md`.

The shared `State` struct (~30 floats: airspeed `U`, accel `Z`, attitude quaternion `E`, gyro rate `H`, earth mag field `N`, plus bias terms `V/C/F/D/K/L`) is held by every algorithm. `Measurement` carries one set of sensor inputs plus per-axis variance accumulators (`varianceAccumulator.go`) used to track noise stats online. Matrix math uses `github.com/skelterjohn/go.matrix`.

Quaternion code (`quaternions.go`) is independent of the algorithm and well-tested (`quaternions_test.go`). The `Regularize` helper in `ahrs_defs.go` is the canonical way to bring roll/pitch/heading back into range.

### `sim/` — offline simulation + replay

A `main` package (not a library). Defines a `Situation` interface implemented by:
- `situationSim.go` — synthesize a flight from a piecewise-linear specification, then add noise/bias to the synthesized sensor data.
- `situationFromFile.go` — replay logged sensor data.

`ahrs_sim.go` wires a `Situation` into a chosen `AHRSProvider` (selectable by CLI flag), advances time, and writes a CSV log via `AHRSLogger` (`ahrs/sensorLogger.go`). This is the primary way to evaluate algorithm changes without flying.

### `sensors/` — hardware drivers

`sensors/defs.go` defines the cross-driver types: `IMUData`, `BMPData`, `IMUSensor` (channels `C`/`CAvg`/`CBuf`), `PressureSensor`, and `IMUCalData` (persisted to `/etc/imu_cal.json`).

Each driver constructs an object that publishes samples on channels (`.C` for the latest reading, `.CBuf` for a 256-deep ring). **All consumer access is through these channels** — there are no synchronous read functions on the driver objects.

Two driver styles coexist:

- **`bmp280/`** — thin adapter over `github.com/westphae/go-iio/bmp280`. The kernel owns the I²C bus and runs the compensation math; this package polls `/sys/bus/iio/devices/...` at ~10 Hz and republishes the readings as `*sensors.BMPData` on the legacy channel API. No `embd` import.
- **`mpu9250/`, `icm20948/`** — userspace I²C drivers via `kidoman/embd`. Each spawns a polling goroutine that bit-bangs registers, runs the InvenSense DMP setup, and publishes `*sensors.IMUData`. The two drivers are near-parallel because both chips use InvenSense's DMP architecture; treat them as siblings, not as one wrapping the other.

The package consolidation under `sensors/` is recent (see commit `3efece4`); some external consumers and test files may still reference the old top-level paths.

#### BMP280 hardware setup

The kernel needs to bind its `bmp280` IIO driver to the chip before goflying can read it. Two ways:

**Persistent (boot-time) — device-tree overlay.** Add to `/boot/firmware/config.txt` (or `/boot/config.txt` on pre-Bookworm) and reboot:

```
dtparam=i2c_arm=on
dtoverlay=i2c-sensor,bmp280,addr=0x76    # or addr=0x77
```

**Per-session — instantiate via sysfs.** Useful when you don't want to reboot or to flip between drivers (e.g. handing the bus back to a userspace embd-based driver):

```sh
sudo modprobe bmp280-i2c
echo bmp280 0x76 | sudo tee /sys/bus/i2c/devices/i2c-1/new_device
# When done:
echo 0x76 | sudo tee /sys/bus/i2c/devices/i2c-1/delete_device
```

Verify the kernel sees it (the `name` file should read `bmp280`):

```sh
ls /sys/bus/iio/devices/
cat /sys/bus/iio/devices/iio:device0/name
```

If you ever want go-iio's buffered/streaming path (not used by goflying today; we poll), also load the hrtimer trigger module and run the process with root (`CAP_SYS_ADMIN` is needed to create the trigger via configfs):

```sh
sudo modprobe iio-trig-hrtimer
```

### `magnetometer/` — magnetometer hard/soft-iron calibration

Three independent algorithms (`magkal_trivial.go`, `magkal_simple.go`, `magkal_kalman.go`), each running as a goroutine that consumes `ahrs.Measurement` on an input channel and emits `MagKalState{K, L}` updates (per-axis scale `K`, offset `L`) on an output channel. `NewMagKal` (`magkal_defs.go`) is the constructor that wires the chosen algorithm into the channel pair.

### `ahrsweb/` — live visualization

`AHRSData` (`ahrs_data.go`) is the on-wire schema: full Kalman state + per-variable uncertainties (`D…` prefix). `Room`/`client` (`room.go`, `client.go`) implement a gorilla/websocket fan-out hub adapted from Mat Ryer's Go Blueprints. `kalman_listener.go` is the bridge that pulls state out of an `AHRSProvider` and pushes `AHRSData` into the room. The `cmd/ahrsweb_server` binary serves HTML/JS from `res/` for browser-side rendering.

Default port is `ahrsweb.Port = 8000`.

### `gdl90Listener/` — telemetry decode

Standalone parser for the iLevil AHRS extension to the GDL-90 protocol over UDP. Independent of the rest of the AHRS pipeline.

## Known TODOs and deferred work

Inventory of in-source TODO/FIXME markers, plus larger modernizations that are intentionally deferred. Update this section as items are addressed.

### Sensor drivers
- `sensors/icm20948/icm20948.go:210` — FIXME: temporary register-bank-2 temp config (testing only).
- `sensors/icm20948/icm20948.go:239` — TODO: use the clock to record actual time instead of a timer.
- `sensors/icm20948/icm20948.go:381` — TODO: `CloseMPU` needs a way to restart the polling goroutine.
- `sensors/mpu9250/mpu9250.go:249` — TODO: use the clock to record actual time instead of a timer.
- `sensors/mpu9250/mpu9250.go:402` — TODO: `CloseMPU` needs a way to restart the polling goroutine.

### AHRS / filter
- `ahrs/ahrs_defs.go:71` — TODO: track separate measurement timestamps for Gyro/Accel, Magnetometer, GPS, Baro.
- `ahrs/ahrs_kalman.go:73, 102` — Kalman filter TODOs (magnetometer usage not yet comprehensive).
- `ahrs/ahrs_test.go:127, 137, 150, 164, 183` — Jacobian and test-loop TODOs.

### Magnetometer
- `magnetometer/magkal_kalman.go:12` — TODO: measure `magNoise` from incoming data rather than hard-coding.

### Sim / telemetry
- `sim/ahrs_sim.go:221` — TODO: log actual state (not just estimated).
- `gdl90Listener/gdl90Listener.go:178` — TODO: implement (parser stub).

### Deferred modernizations (not yet started)
- Replace `github.com/skelterjohn/go.matrix` (last release ~2013) with `gonum.org/v1/gonum/mat`. Used in `ahrs/ahrs_kalman*.go`, `ahrs/ahrs_simple.go`, `ahrs/ahrs_state.go`, `sim/situationSim.go`, `sim/situationFromFile.go`. AHRS tests should catch numerical regressions.
- Replace `github.com/kidoman/embd` (stale since ~2017) with `periph.io/x/conn/v3` or move to kernel IIO via `github.com/westphae/go-iio`. BMP280 is already migrated to go-iio; ICM-20948 and MPU-9250 are still on embd. Doing the remaining two would let us drop the `westphae/embd` replace entirely (currently needed because upstream embd panics on modern Raspberry Pi OS kernel strings).
- Add `context.Context` and explicit shutdown to long-running goroutines, especially in `magnetometer/research/calibration.go` (HTTP handlers spawning unbounded loops, channels never closed).
- Add CI (GitHub Actions) — at minimum `go build ./...`, `go vet ./...`, and `go test ./ahrs/...` on Linux. Sensor `test/` binaries should be built but not run.
- Expand test coverage: only `ahrs/` has any `*_test.go` files (~5% of source files). `ahrsweb/`, `gdl90Listener/`, `magnetometer/`, `sim/`, and all of `sensors/` are untested.
- Modernize `math/rand.Seed()` usage in `ahrs/ahrs_test.go` (Seed is a no-op since Go 1.20; switch to `rand.New(rand.NewSource(...))` for per-test isolation, or drop the seed calls).
- Refactor `main` packages under `sim/`, `sensors/*/test/`, `magnetometer/research/` into a `cmd/` layout for cleaner module structure.
