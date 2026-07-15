# Humanoid Teleoperation Arm – Progression Report  
**Last Updated:** May 2026

---

# Project Overview

This project is a fully custom humanoid teleoperation robotic arm and anthropomorphic robotic hand platform designed for:

- Real-time human teleoperation
- Human motion replication
- Research in telepresence and haptics
- Collaborative robotics experimentation
- Force interaction experiments
- Modular robotic control development

The system combines:

- A 7-DOF robotic arm
- A 6-DOF robotic hand
- Wearable IMU tracking
- Flex sensor glove input
- Wi-Fi low-latency communication
- RS-485 Dynamixel motor network
- Custom embedded control architecture
- Fully 3D printed mechanical structure

The project architecture follows principles similar to open-source humanoid research platforms such as Reachy fileciteturn0file0L146-L164 while extending functionality toward full teleoperation and anthropomorphic hand control.

---

# Current Project Status

## Overall Status
The project is mechanically complete and operational at the subsystem level.

Main robotic arm:
- Mechanical structure completed
- Electronics integrated
- Dynamixel communication operational
- IMU tracking operational
- Inverse kinematics implemented
- Wrist control implemented
- Teleoperation functioning

Robotic hand:
- Mechanical structure largely completed
- Electronics completed
- Encoder system implemented
- ESP32-S3 hand controller integrated
- Homing and calibration infrastructure implemented
- Firmware under active development/testing

Main current focus:
- Communication reliability
- Motion smoothness
- Gear durability
- Hand integration
- Final system stability

---

# Mechanical System

## Arm Structure
The robotic arm is:
- Fully 3D printed
- PETG structural construction
- Modular link-based design
- Bearing-supported rotational joints
- Dynamixel-based actuation system

The design philosophy emphasizes:
- Modularity
- Repairability
- Simple replacement of parts
- Easy iteration and redesign

The project uses:
- Metal four-point contact bearings
- Large reduction ratios
- Thick wall reinforced printed parts
- Printed helical gears

## Arm Degrees of Freedom
The arm currently contains:
- Shoulder yaw
- Shoulder pitch
- Shoulder roll
- Elbow flexion
- Forearm rotation
- Wrist yaw
- Wrist pitch

The system effectively operates as a 7-DOF teleoperation platform.

## Gear System
Recent revisions:
- Gear module increased from 1.0 mm to 1.5 mm
- Helical gears used to reduce backlash
- Current focus is improving tooth strength and preventing deformation under load

Current issue under investigation:
- Tooth skipping due to gear deformation

---

# Robotic Hand

## Hand Architecture
The robotic hand is inspired by linkage-driven anthropomorphic robotic hand designs such as the ALARIS hand fileciteturn0file1L1-L17 while using a custom architecture optimized for teleoperation.

The hand contains:
- 6 motorized axes
- Worm gear driven fingers
- Encoder feedback
- Homing switches
- ESP32-S3 dedicated controller

## Finger Drive System
Features:
- Worm gear + rack mechanisms
- Non-backdrivable finger actuation
- 100:1 geared DC motors
- Position tracking through encoders
- Compact modular finger assemblies

The worm-drive approach was selected for:
- High holding torque
- Mechanical stability
- Compact packaging
- Passive holding without constant power

## Hand Sensors
Integrated:
- Hall encoder system
- Flex sensors
- Homing switches

Flex sensors used:
- SpectraSymbol style bend sensors
- Approximately 25 kΩ flat resistance
- 45–125 kΩ under bending fileciteturn0file3L1-L31

---

# Electronics Architecture

## Main Controllers

### Teensy 4.1
Primary control processor:
- Kinematics
- Motion control
- Dynamixel communication
- Sensor fusion
- Teleoperation logic

### Arduino Nano ESP32 Units
Three Nano ESP32 boards used for:
- User-side IMU acquisition
- Robot-side communication relay
- Auxiliary support tasks

### ESP32-S3 Hand Controller
Dedicated hand controller responsible for:
- Finger motor control
- Encoder acquisition
- Homing switches
- Multiplexer handling
- Communication with main robot

Hand controller pinout and architecture are documented in the project files fileciteturn0file8L1-L66

---

# Sensor System

## IMU System
Primary motion sensing uses:
- BNO086 IMUs
- SPI communication
- Quaternion-based orientation tracking

Configuration:
- 3 user-side IMUs
- 1 robot reference IMU

The system currently transmits:
- Body axes orientation matrices
- Quaternion-derived rotational data
- 50 Hz reporting frequency

## Force Sensing
The system includes:
- Flexible FSR sensors
- Palm/finger force detection
- Future collaborative interaction experiments

Planned functionality:
- External force estimation
- Compliance behaviors
- Basic cobot interaction

---

# Communication Architecture

## User ↔ Robot Link
Communication method:
- Wi-Fi
- UDP packet streaming

Configuration:
- SSID: `robot`
- Real-time orientation streaming

## Robot Internal Communication

### RS-485 Network
Used for:
- Dynamixel motor chain

Current status:
- 3 Mbps motor communication achieved

### UART Links
High-speed UART communication implemented between:
- ESP32 units
- Teensy controller

Current operational baud rates:
- 921600 bps between ESP32 and Teensy
- 3 Mbps on Dynamixel network

Recent work focused heavily on:
- Packet integrity
- Latency reduction
- Communication synchronization
- CPU load balancing

---

# Software System

## Core Features Implemented

### Inverse Kinematics
Implemented:
- Full arm inverse kinematics
- Relative orientation tracking
- Quaternion-based frame calculations
- Joint-space motion mapping

### Teleoperation
Operational:
- Real-time arm mirroring
- User arm orientation tracking
- Relative motion reconstruction

### Motion Filtering
Implemented:
- Low-pass filtering
- Slew-rate limiting
- Motion smoothing
- Jitter reduction

### Wrist Control
Implemented:
- Wrist yaw
- Wrist pitch
- Forearm roll synchronization

### Motor Control
Implemented:
- Dynamixel Protocol 1.0 and 2.0 mixed network
- Extended position mode
- Startup calibration
- Motor offset capture

---

# Current Technical Challenges

## Communication Stability
Main current software focus:
- Eliminating sudden jumps
- Improving UART robustness
- Preventing packet corruption
- Stabilizing multi-IMU synchronization

## Gear Durability
Current mechanical focus:
- Preventing gear skipping
- Increasing gear tooth strength
- Minimizing backlash

## Motion Smoothness
Current tuning focus:
- Filtering refinement
- Timing synchronization
- Communication latency reduction

---

# Power Architecture

## Main Power
Primary PSU:
- LRS-600-24
- 24 V
- 600 W

## Power Rails
Current distribution:
- 24 V Dynamixel supply
- Dual 24→12 V buck converters
- 12→5 V logic rail
- Dedicated 3.3 V regulation

System hardware summary is documented in the project files fileciteturn0file2L1-L25

---

# Research and Future Development

## Planned Features
Future goals include:
- Haptic feedback
- Force-reflective teleoperation
- Improved collaborative robotics behavior
- Enhanced hand dexterity
- Full hand calibration system
- Autonomous motion assistance
- Trajectory planning improvements

## Potential Research Applications
The platform may be used for:
- Human-robot interaction research
- Telepresence systems
- Assistive robotics
- Prosthetics research
- Shared-control systems
- Motion learning experiments

---

# Important Current Notes

- Mechanical assembly is largely complete
- Arm control system is functional
- Teleoperation already works
- Hand system is entering final integration phase
- Communication stability remains the highest software priority
- Gear reliability remains the highest mechanical priority
- The project has transitioned from conceptual development into integration and stabilization

---

# File References

Key project files currently available:

- Main Teensy control firmware fileciteturn0file7L1-L50
- Robot-side ESP32 firmware fileciteturn0file5L1-L50
- User-side ESP32 firmware fileciteturn0file6L1-L50
- Hand controller pinout fileciteturn0file8L1-L66
- Hardware summary fileciteturn0file2L1-L25
- Flex sensor datasheet fileciteturn0file3L1-L31