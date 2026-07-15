# Firmware

## Controllers

| Directory | Target board | Primary responsibility |
|---|---|---|
| `arm-controller-teensy41` | Teensy 4.1 | Kinematics, filtering, safety gating and Dynamixel commands |
| `robot-communication-nano-esp32` | Arduino Nano ESP32 | Robot access point, UDP receive, reference IMU and Teensy bridge |
| `user-wearable-nano-esp32` | Arduino Nano ESP32 | Three wearable IMUs, flex sensors and UDP transmission |
| `hand-controller-esp32s3` | ESP32-S3 | Six DC motors, encoders, PID, homing and telemetry |

## Reproducibility work still required

Before declaring a reproducible release, record:

- Arduino IDE or PlatformIO version
- Board-core versions
- Library names and versions
- Board selection and USB mode
- Pinout revision
- Required compile flags
- Flashing procedure
- Calibration data and safe startup procedure

## Credentials

The imported V1 firmware contains development Wi-Fi credentials. Keep the repository private and move credentials into ignored local configuration files before any public release.
