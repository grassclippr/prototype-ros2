# Rover Navigation Bring-up

This package keeps navigation bring-up incremental and script-first.

## First checks

Start the existing robot stack, then run the navigation bridge/localization:

```bash
make core-hw
make ros ARGS='launch rover_navigation localization.launch.py use_imu:=false'
```

Use `use_imu:=true` once an IMU driver is publishing `sensor_msgs/Imu` on `/imu/data`.

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
