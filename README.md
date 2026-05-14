# goflying

Go library implementing an Attitude and Heading Reference System (AHRS) for the
[Stratux](https://github.com/cyoung/stratux) ADS-B receiver, along with sensor
drivers (BMP280 barometer, ICM-20948 IMU), offline simulation, magnetometer
calibration, and a websocket visualization server.

See [CLAUDE.md](CLAUDE.md) for the full architecture overview and known TODOs.

## Building

Requires Go 1.22 or newer.

```sh
go mod tidy
go build ./...
go test ./ahrs/...
```

The sensor `test/` binaries (`./sensors/bmp280/test`, `./sensors/icm20948/test`)
compile on any platform but only function on a Raspberry Pi (or similar SBC)
with the corresponding chip wired to the I²C bus.

## Running on a Raspberry Pi

Enable I²C bus 1 in `raspi-config` (or `dtparam=i2c_arm=on` in
`/boot/firmware/config.txt`). After that:

- **BMP280** is read through the kernel's IIO driver (no direct I²C bit-banging
  from userspace). Bind the driver one of two ways:

  ```sh
  # Persistent: add to /boot/firmware/config.txt and reboot
  dtoverlay=i2c-sensor,bmp280,addr=0x76   # or addr=0x77

  # Per-session: instantiate via sysfs (no reboot)
  sudo modprobe bmp280-i2c
  echo bmp280 0x76 | sudo tee /sys/bus/i2c/devices/i2c-1/new_device
  # Tear down with:
  echo 0x76 | sudo tee /sys/bus/i2c/devices/i2c-1/delete_device
  ```

  Verify with `cat /sys/bus/iio/devices/iio:device0/name` — it should print
  `bmp280`. Then `go run ./sensors/bmp280/test` prints a CSV of live readings.

- **ICM-20948** uses an out-of-tree kernel module that lives in a sibling
  repo, [`westphae/icm20948-mod`](https://github.com/westphae/icm20948-mod).
  Build and install it, then load the hrtimer trigger module that goflying's
  streaming capture needs:

  ```sh
  # Persistent: build + install dtbo + reboot
  cd ../icm20948-mod
  make && sudo make install
  sudo reboot

  # Per-session: insmod + bind via sysfs (no reboot)
  sudo insmod ./icm20948.ko
  echo icm20948 0x68 | sudo tee /sys/bus/i2c/devices/i2c-1/new_device   # or 0x69
  # Tear down with:
  echo 0x68 | sudo tee /sys/bus/i2c/devices/i2c-1/delete_device
  sudo rmmod icm20948

  # Always required (once per boot) before streaming:
  sudo modprobe iio-trig-hrtimer
  ```

  To make the trigger module load automatically on every boot, drop a one-line
  file into `/etc/modules-load.d/`:

  ```sh
  echo iio-trig-hrtimer | sudo tee /etc/modules-load.d/iio-trig-hrtimer.conf
  ```

  (The BMP280 driver autoloads via the device-tree overlay above, so no
  equivalent file is needed for it.)

  Verify with `cat /sys/bus/iio/devices/iio:device*/name` — one of them should
  print `icm20948`. Then `sudo go run ./sensors/icm20948/test` prints a live
  TUI of accel/gyro/mag/temp readings. Root is required because go-iio
  creates the hrtimer trigger via configfs.

## Consumers

Two downstream projects live alongside this repo in GOPATH:

- **[magkal](https://github.com/westphae/magkal)** — Go-modules build that
  uses goflying via `replace github.com/westphae/goflying => ../goflying`.
  Mostly exercises `sensors/icm20948`.
- **[stratux](https://github.com/cyoung/stratux)** — GOPATH-style; consumes
  goflying as a git submodule. Pinned to a specific commit, so canonical
  goflying changes only reach stratux when its submodule pointer is bumped.
