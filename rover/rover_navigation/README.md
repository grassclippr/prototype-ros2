# Rover Navigation Bring-up

This package keeps navigation bring-up incremental and script-first.

## First checks

Start the existing robot stack, then run the navigation bridge/localization:

```bash
make core-hw
make ros ARGS='launch rover_navigation localization.launch.py use_imu:=false'
```

Use `use_imu:=true` once the ICM20948 is reachable on `/dev/i2c-1`. The localization launch now starts the IMU driver directly and publishes `sensor_msgs/Imu` on `/imu/data`.

On the Pi, confirm the sensor is present first:

```bash
sudo modprobe i2c-dev
sudo i2cdetect -y 1
```

The Adafruit ICM20948 breakout should appear at `0x69`.

If you are starting the hardware stack locally with `make core-hw`, set `IMU_I2C_DEV=/dev/i2c-1` in `.env` on the Pi so compose passes the device through to the `core` container. The deploy scripts already default to that path.

Then start localization with the IMU enabled:

```bash
make core-hw
make ros ARGS='launch rover_navigation localization.launch.py use_imu:=true'
```

The initial integration pass uses gyro + accelerometer only. Magnetometer heading is intentionally deferred until the basic localization path is stable.

When GNSS is enabled, the baseboard now publishes raw sentences on `/baseboard/nmea_sentence_raw`. A Pi-side restamp node republishes them on `/nmea_sentence` with Pi receipt time in `header.stamp` while `nmea_navsat_driver` continues to expose GNSS UTC via `/time_reference`.

## Validation commands

```bash
make nav-check-localization
make nav-check-imu
make nav-check-gnss
make nav-check-stack
```

## Nav2 dry run

```bash
make ros ARGS='launch rover_navigation bringup.launch.py use_imu:=false start_nav2:=true'
make nav-check-stack
make nav-send-goal X=0.5 Y=0.0 YAW=0.0
```

The default Nav2 configuration starts only the runtime navigation nodes, uses empty rolling costmaps, low speed limits, and an identity `map -> odom` transform for early local-goal tests. Disable `publish_map_odom_identity` later when map/GNSS localization owns `map -> odom`.
