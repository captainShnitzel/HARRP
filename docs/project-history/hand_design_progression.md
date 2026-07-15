# Complete Pin and Connection List (From Project Source Files)

## User-Side Arduino Nano ESP32 (3× IMU transmitter)

### BNO086 IMU Connections
From the user-side firmware: fileciteturn4file6L10-L27

| Signal | Pin |
|---|---|
| IMU1 CS | GPIO 10 |
| IMU2 CS | GPIO 9 |
| IMU3 CS | GPIO 8 |
| IMU1 INT | GPIO 7 |
| IMU2 INT | GPIO 6 |
| IMU3 INT | GPIO 3 |
| Shared RESET | GPIO 4 |

### SPI Bus
fileciteturn4file6L284-L285

| SPI Signal | Pin |
|---|---|
| SCK | GPIO 13 |
| MISO | GPIO 12 |
| MOSI | GPIO 11 |

### RGB Debug LED
fileciteturn4file6L22-L24

| LED | Pin |
|---|---|
| Red | GPIO 14 |
| Green | GPIO 15 |
| Blue | GPIO 16 |

### Wi-Fi
fileciteturn4file6L28-L33

| Parameter | Value |
|---|---|
| SSID | `robot` |
| Password | `12345678` |
| UDP Port | 5005 |
| Robot IP | 192.168.4.1 |

---

# Robot-Side Arduino Nano ESP32

### BNO086 Reference IMU
fileciteturn4file7L16-L28

| Signal | Pin |
|---|---|
| CS | GPIO 10 |
| INT | GPIO 4 |
| RESET | GPIO 5 |

### RGB Debug LED
fileciteturn4file7L16-L19

| LED | Pin |
|---|---|
| Red | GPIO 14 |
| Green | GPIO 15 |
| Blue | GPIO 16 |

### UART to Teensy
fileciteturn4file7L81-L83

| Connection | Interface |
|---|---|
| ESP32 → Teensy | Serial0 UART |

### Wi-Fi Access Point
fileciteturn4file7L9-L13

| Parameter | Value |
|---|---|
| AP SSID | `robot` |
| AP Password | `12345678` |
| UDP Listen Port | 5005 |

---

# Teensy 4.1 Main Controller

### Serial Interfaces
fileciteturn4file8L34-L39

| Function | Port |
|---|---|
| Debug USB Serial | Serial |
| ESP32 Reference UART | Serial1 |
| Dynamixel Bus | Serial2 |

### RS-485 Direction Pin
fileciteturn4file8L43-L44

| Signal | Pin |
|---|---|
| RS-485 DIR | GPIO 28 |

### Dynamixel Motor IDs
fileciteturn4file8L54-L60

| Joint | ID |
|---|---|
| Q1 Shoulder Yaw | 1 |
| Q2 Shoulder Pitch | 2 |
| Q3 Shoulder Roll | 3 |
| Q4 Elbow | 4 |
| Q5 Forearm Rotation | 5 |
| Q6 Wrist Yaw | 6 |
| Q7 Wrist Pitch | 7 |

### Communication Speeds
fileciteturn4file8L47-L49

| Interface | Speed |
|---|---|
| Debug Serial | 2,000,000 |
| ESP32 UART | 2,000,000 |
| Dynamixel RS-485 | 3,000,000 |

---

# ESP32-S3 Hand Controller Connections

## Motor Driver Bus Architecture
From progression reports and hand controller summary: fileciteturn4file5L129-L145 fileciteturn4file10L18-L33

### Drivers
| Component | Qty |
|---|---|
| TB6612FNG Dual Drivers | 3 |
| Total Motors Controlled | 6 |

### Motor System
| Motor | Type |
|---|---|
| Motors 1-6 | 12V Micro Metal Gearmotors |

---

# Hand Encoder Assignments

From project source clarification: fileciteturn4file5L287-L289

| Encoder Pin | Function |
|---|---|
| GPIO 35 | Thumb Rotation Encoder A |
| GPIO 36 | Thumb Rotation Encoder B |
| GPIO 37 | Thumb Flex Encoder |
| GPIO 38 | Finger 1 Encoder |
| GPIO 39 | Finger 2 Encoder |
| GPIO 40 | Finger 3 Encoder |
| GPIO 41 | Finger 4 Encoder |

---

# Hand Logic Power Distribution

From updated progression report: fileciteturn4file10L45-L55

| Rail | Purpose |
|---|---|
| ESP32-S3 3.3V Rail #1 | Encoders |
| ESP32-S3 3.3V Rail #2 | MUX + Homing Switches |
| Shared Ground | Entire hand logic system |

---

# Hand Sensor / MUX System

From project descriptions: fileciteturn4file5L139-L145

| Component | Description |
|---|---|
| Multiplexer | 16-channel MUX |
| Connected To | Flex sensors + homing switches |
| Logic Voltage | 3.3V |

---

# Glove Connector Wiring

From glove system notes: fileciteturn4file5L190-L197

| Pin Usage | Count |
|---|---|
| VCC Pins | 6 |
| Ground Pins | 6 |
| MUX Signal Pins | 6 |

| Connector | Type |
|---|---|
| Glove Connector | 25-pin multi-pin connector |

---

# Flex Sensor Electrical Characteristics

From flex sensor datasheet: fileciteturn4file3L1-L31

| Parameter | Value |
|---|---|
| Flat Resistance | 25 kΩ |
| Bent Resistance | 45–125 kΩ |
| Supply Style | Voltage Divider |
| Sensor Length | ~55 mm active |

---

# Main Power System

From hardware summary: fileciteturn4file2L1-L25

| Rail | Hardware |
|---|---|
| Main PSU | LRS-600-24 |
| Main Voltage | 24V |
| 12V Rail A | Dynamixel MX motors |
| 12V Rail B | Palm motors |
| 5V Rail | Logic |
| 3.3V Rail | S9V11F3S5 regulator |

---

# RS-485 / Dynamixel Network

From hardware summary: fileciteturn4file2L1-L25

| Component | Qty |
|---|---|
| U2D2 + Power Hub | 3 |

| Motor Type | Voltage |
|---|---|
| XH540-V270-R | 24V |
| MX-64R | 12V |
| MX-28R | 12V |

---

# GPIO Pull-Down Recommendation

From project electrical notes: fileciteturn4file9

| Component | Value |
|---|---|
| Pull-down resistor | 10 kΩ |

Used for:
- Teensy ↔ ESP32 signaling lines
- Stable boot states
- Non-floating GPIO inputs