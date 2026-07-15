# HARRP

**Human-Assistive Robotic Research Platform**

HARRP is a modular humanoid robotic research platform developed for human–robot control, teleoperation, assistive robotics, haptics, and future biosignal/AI-assisted control research.

The current repository begins with the completed HARRP V1 bachelor’s-project implementation:

- 7-DOF humanoid robotic arm
- 6-axis anthropomorphic robotic hand
- Wearable IMU and flex-sensor input
- Wi-Fi/UDP user-to-robot communication
- Teensy 4.1 real-time motion controller
- Mixed-protocol Dynamixel RS-485 network
- ESP32-S3 hand controller with DC-motor position control and homing

## Repository layout

```text
firmware/
  arm-controller-teensy41/          Main arm kinematics and Dynamixel control
  robot-communication-nano-esp32/   Robot-side Wi-Fi, IMU and Teensy bridge
  user-wearable-nano-esp32/         Wearable IMUs, glove sensors and UDP sender
  hand-controller-esp32s3/          Hand motors, encoders, PID and homing
hardware/
  bom/                              Bill of materials
  diagrams/                         Electrical and logic diagrams
  summaries/                        Hardware summary documents
docs/
  architecture/                     System architecture documentation
  engineering-notes/                Failure analysis and troubleshooting
  project-history/                  V1 development and presentation history
references/                          External source index; papers are not committed
tools/                               Future test, calibration and support scripts
```

## Versioning policy

- `main` contains the current integrated development state.
- The final bachelor’s-project baseline should be tagged `v1.0.0`.
- HARRP V2 development should continue in this repository so design history remains traceable.
- Experimental work should use short-lived branches such as `feature/rtos-migration` or `feature/jetson-interface`.

## Build status

The firmware sources were imported from the final HARRP V1 working files. Board packages, Arduino libraries, and exact toolchain versions still need to be documented before builds are fully reproducible.

See [`firmware/README.md`](firmware/README.md).

## Safety notice

This project controls a high-torque robotic arm. Firmware changes must be tested with motor power isolated or with the arm mechanically secured before powered operation. Do not bypass startup gating, communication watchdogs, position limits, current limits, or emergency-stop provisions.

## Publication and licensing

The repository should remain **private** until all of the following are reviewed:

1. University and laboratory intellectual-property requirements.
2. Commercialization and patent considerations.
3. Third-party code and hardware-license compatibility.
4. Removal of test credentials, personal data, and non-redistributable documents.

No open-source license has been selected yet. Until a license is deliberately added, no permission is granted to copy, modify, or redistribute the repository.
