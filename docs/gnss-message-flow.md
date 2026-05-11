# GNSS And RTCM Message Flow

This note shows how GNSS data and RTCM corrections move through the rover system.

The important split is:

- `RTCM in`: correction data sent to the rover GNSS receiver
- `GNSS fix out`: position/time data sent from the rover GNSS receiver into ROS

## Big Picture

```mermaid
flowchart LR
    subgraph BaseStation["Base station"]
        BGNSS["Base GNSS receiver"]
        BMCU["Base MCU"]
        BGNSS -->|"raw GNSS measurements"| BMCU
        BMCU -->|"RTCM corrections"| Radio["ESP-NOW / radio link"]
    end

    subgraph Rover["Rover"]
        Radio -->|"RTCM corrections"| RMCU["Rover MCU / baseboard"]
        RMCU -->|"RTCM bytes over UART"| RGNSS["Rover GNSS receiver"]
        RGNSS -->|"NMEA sentences over UART"| RMCU

        subgraph RawPath["Raw NMEA ROS path"]
            RMCU -->|"GGA/RMC text on /baseboard/nmea_sentence_raw"| Restamper["Pi nmea_sentence_restamper"]
            Restamper -->|"restamped NMEA on /nmea_sentence"| NMEADriver["Pi nmea_navsat_driver"]
            NMEADriver -->|"/fix"| ROSFix1["ROS consumers"]
            NMEADriver -->|"/time_reference"| ROSTime1["ROS consumers"]
        end

        subgraph StructuredPath["Structured ROS path"]
            RMCU -->|"compact fix on /baseboard/gnss_fix"| FixBridge["Pi gnss_fix_bridge"]
            FixBridge -->|"/fix"| ROSFix2["ROS consumers"]
            FixBridge -->|"/time_reference"| ROSTime2["ROS consumers"]
        end

        ROSFix1 --> Nav["robot_localization / navsat_transform / Nav2"]
        ROSTime1 --> Nav
        ROSFix2 --> Nav
        ROSTime2 --> Nav
    end
```

## What Each Part Does

### 1. Base station side

- The base GNSS receiver produces data that the base MCU uses to generate RTCM corrections.
- The base MCU sends those RTCM corrections to the rover over the radio link.

This path is about **improving the rover's GNSS solution**. It is not the same as the ROS navigation path.

### 2. Rover correction input path

- The rover MCU receives RTCM corrections over radio.
- The rover MCU forwards the RTCM bytes to the rover GNSS receiver over `Serial2`.
- The rover GNSS receiver uses those corrections internally to compute a better fix.

This is the **RTCM in** path.

### 3. Rover GNSS output path

- The rover GNSS receiver emits NMEA sentences such as `GGA` and `RMC`.
- The rover MCU reads those sentences and decides what to forward toward ROS.

This is the **GNSS fix out** path.

## Two Output Styles

### Option A: Raw NMEA path

This is the debug and diagnostics path.

```mermaid
flowchart LR
    GNSS["Rover GNSS"] -->|"NMEA over UART"| MCU["Rover MCU"]
    MCU -->|"/baseboard/nmea_sentence_raw\nGGA/RMC text"| Restamper["nmea_sentence_restamper.py"]
    Restamper -->|"/nmea_sentence"| Driver["nmea_navsat_driver"]
    Driver -->|"/fix"| Fix["ROS fix users"]
    Driver -->|"/time_reference"| Time["ROS time users"]
```

Characteristics:

- preserves raw sentence text
- easy to debug with standard NMEA tools
- higher bandwidth and framing overhead on the MCU to Pi transport
- parses the GNSS data twice: once implicitly in sentence selection, once on the Pi

### Option B: Structured fix path

This is the preferred default path going forward.

```mermaid
flowchart LR
    GNSS["Rover GNSS"] -->|"NMEA over UART"| MCU["Rover MCU"]
    MCU -->|"/baseboard/gnss_fix\ncompact structured fields"| Bridge["gnss_fix_bridge.py"]
    Bridge -->|"/fix"| Fix["ROS fix users"]
    Bridge -->|"/time_reference"| Time["ROS time users"]
```

Characteristics:

- sends only the fields ROS needs
- lower transport cost than raw NMEA
- easier to rate-limit cleanly
- better fit for dead reckoning plus periodic GNSS correction
- less convenient for low-level NMEA debugging

## Why RTCM Does Not Need To Change This Much

RTCM is correction data flowing **into** the rover GNSS receiver.

The GNSS output path is separate:

- RTCM in improves the solution quality
- GNSS fix out tells ROS what the current solution is

So adding RTK mostly changes the **quality of the fix**, not the basic shape of the ROS topics.

What may need to expand later is the structured message content, for example:

- RTK fixed vs RTK float
- correction age
- horizontal / vertical accuracy
- receiver-specific quality indicators

## Current Structured Message Content

The current `GnssFix` message carries:

- UTC date and time
- latitude
- longitude
- altitude
- HDOP
- satellite count
- fix quality

That is enough for a first compact ROS path, but not yet a full RTK status model.

## Publish Rate Guidance

For navigation with wheel dead reckoning:

- wheel/IMU odometry should run at a higher rate
- GNSS can usually be slower
- `5 Hz` is a reasonable first target for GNSS fix publication

That is why the structured path is currently set up to be comfortable with a slower publish period than raw NMEA.

## Practical Interpretation

If you want the simplest mental model:

1. The base station sends RTCM corrections to the rover.
2. The rover GNSS uses those corrections to improve its fix.
3. The rover MCU forwards the resulting fix to the Pi.
4. ROS consumes `/fix` and `/time_reference`.

The main design choice is only **how step 3 is encoded**:

- raw NMEA text
- or a compact structured fix message
