BNO08x Multi-SPI Library
=======================

Overview
--------
This is a modified version of the SparkFun BNO08x Arduino library
that allows multiple BNO08x IMUs to operate over a shared SPI bus.

Each sensor uses its own Chip Select (CS) line, enabling multiple
devices to run independently on the same SPI interface.


Features
--------
- Multiple IMUs on one SPI bus
- Independent sensor instances
- Compatible with Teensy / ESP32
- Based on SparkFun BNO08x library


Requirements
------------
- BNO08x IMU (BNO085 / BNO086)
- Microcontroller with SPI support
- Separate CS pin for each IMU

Recommended:
- Separate INT pin per sensor
- Separate RST pin per sensor


Wiring
------

Shared SPI Lines:
  MOSI -> common
  MISO -> common
  SCK  -> common

Per IMU:
  CS  -> unique pin (required)
  INT -> optional (recommended)
  RST -> optional (recommended)


Basic Usage
-----------

Include library:
  #include "BNO08x_MultiSPI.h"

Create instances:
  BNO08x imu1;
  BNO08x imu2;

Initialize:
  imu1.beginSPI(CS1, INT1, RST1);
  imu2.beginSPI(CS2, INT2, RST2);

Enable reports:
  imu1.enableRotationVector(50);
  imu2.enableRotationVector(50);

Read data:
  if (imu1.dataAvailable()) {
      imu1.getQuat();
  }

  if (imu2.dataAvailable()) {
      imu2.getQuat();
  }


Notes
-----
- Each IMU must have a unique CS pin
- Keep SPI wiring short and stable
- Add decoupling capacitors near each sensor
- If a sensor fails to initialize, reset it individually


Based On
--------
SparkFun BNO08x Arduino Library


Status
------
Tested with multiple IMUs running simultaneously over SPI.