//v1.0 
#include <Arduino.h>
#include <SPI.h>
#include "BNO08x_MultiSPI.h"
#include <WiFi.h>
#include <WiFiUdp.h>

BNO08x imu;

const char* AP_SSID = "robot";
const char* AP_PASS = "12345678";
const uint16_t LISTEN_PORT = 5005;

WiFiUDP udp;

// LEDs
#define ledRed   14
#define ledGreen 15
#define ledBlue  16

// BNO086 on Nano ESP32 (same wiring as your working user-side setup)
#define BNO08X_CS   10
#define BNO08X_INT  4
#define BNO08X_RST  5

bool imu_ok = false;
bool wifi_ok = false;

// ============================================================
// Dual 7-segment debug display
// 2x SN74LS47N, static BCD/raw-glyph drive
// Board connector:
//   A0 A1 A2 A3 = right / ones digit BCD A B C D
//   A4 A5 A6 A7 = left  / tens digit BCD A B C D
// ============================================================

static constexpr uint8_t DISP_ONES_A = A0;
static constexpr uint8_t DISP_ONES_B = A1;
static constexpr uint8_t DISP_ONES_C = A2;
static constexpr uint8_t DISP_ONES_D = A3;

static constexpr uint8_t DISP_TENS_A = A4;
static constexpr uint8_t DISP_TENS_B = A5;
static constexpr uint8_t DISP_TENS_C = A6;
static constexpr uint8_t DISP_TENS_D = A7;

static constexpr uint8_t DISP_ONES_PINS[4] = {
  DISP_ONES_A,
  DISP_ONES_B,
  DISP_ONES_C,
  DISP_ONES_D
};

static constexpr uint8_t DISP_TENS_PINS[4] = {
  DISP_TENS_A,
  DISP_TENS_B,
  DISP_TENS_C,
  DISP_TENS_D
};

enum DebugGlyph : uint8_t {
  GLYPH_0     = 0,
  GLYPH_1     = 1,
  GLYPH_2     = 2,
  GLYPH_3     = 3,
  GLYPH_4     = 4,
  GLYPH_5     = 5,
  GLYPH_6     = 6,
  GLYPH_7     = 7,
  GLYPH_8     = 8,
  GLYPH_9     = 9,
  GLYPH_10    = 10,
  GLYPH_11    = 11,
  GLYPH_12    = 12,
  GLYPH_13    = 13,
  GLYPH_14    = 14,
  GLYPH_BLANK = 15
};

enum DebugCode : uint8_t {
  DBG_BOOT            = 0,  // 00 general boot
  DBG_WAIT_USER       = 1,  // 01 waiting for user side
  DBG_USER_CONNECTED  = 2,  // 02 user packets detected
  DBG_WAIT_TEENSY     = 3,  // 03 waiting for Teensy ready
  DBG_TEENSY_READY    = 4,  // 04 Teensy ready
  DBG_ROBOT_IMU_OK    = 5,  // 05 robot IMU OK
  DBG_ALL_IMUS_ACTIVE = 6,  // 06 all user IMUs fresh
  DBG_TELEOP_ACTIVE   = 7,  // 07 active
  DBG_PAUSED          = 8,  // 08 paused / disabled
  DBG_STANDBY         = 9   // 09 standby
};

enum DebugFault : uint8_t {
  FAULT_GENERAL        = 0,  // E0
  FAULT_COMM           = 1,  // E1
  FAULT_IMU            = 2,  // E2
  FAULT_TEENSY         = 3,  // E3
  FAULT_DYNAMIXEL      = 4,  // E4
  FAULT_HAND           = 5,  // E5
  FAULT_HAND_CAL       = 6,  // E6
  FAULT_SAFETY_LOCKOUT = 7,  // E7
  FAULT_POWER_HW       = 8,  // E8
  FAULT_FATAL          = 9   // E9
};

static constexpr DebugGlyph FAULT_PREFIX = GLYPH_13;

static DebugGlyph requestedLeftGlyph  = GLYPH_BLANK;
static DebugGlyph requestedRightGlyph = GLYPH_BLANK;

static DebugGlyph currentLeftGlyph  = GLYPH_BLANK;
static DebugGlyph currentRightGlyph = GLYPH_BLANK;

static bool debugDisplayForceRefresh = true;

void writeRawGlyph(const uint8_t pins[4], DebugGlyph glyph) {
  uint8_t value = ((uint8_t)glyph) & 0x0F;

  digitalWrite(pins[0], (value & 0x01) ? HIGH : LOW);  // BCD A
  digitalWrite(pins[1], (value & 0x02) ? HIGH : LOW);  // BCD B
  digitalWrite(pins[2], (value & 0x04) ? HIGH : LOW);  // BCD C
  digitalWrite(pins[3], (value & 0x08) ? HIGH : LOW);  // BCD D
}

void initDebugDisplay() {
  for (int i = 0; i < 4; i++) {
    pinMode(DISP_ONES_PINS[i], OUTPUT);
    pinMode(DISP_TENS_PINS[i], OUTPUT);

    digitalWrite(DISP_ONES_PINS[i], LOW);
    digitalWrite(DISP_TENS_PINS[i], LOW);
  }

  requestedLeftGlyph  = GLYPH_0;
  requestedRightGlyph = GLYPH_0;

  currentLeftGlyph  = GLYPH_BLANK;
  currentRightGlyph = GLYPH_BLANK;

  debugDisplayForceRefresh = true;
}

void requestDebugGlyphs(DebugGlyph left, DebugGlyph right) {
  requestedLeftGlyph = left;
  requestedRightGlyph = right;
}

void requestDebugCode(DebugCode code) {
  uint8_t value = (uint8_t)code;

  requestDebugGlyphs(
    (DebugGlyph)(value / 10),
    (DebugGlyph)(value % 10)
  );
}

void requestDebugFault(DebugFault fault) {
  requestDebugGlyphs(
    FAULT_PREFIX,
    (DebugGlyph)((uint8_t)fault % 10)
  );
}

void updateDebugDisplay() {
  if (!debugDisplayForceRefresh &&
      requestedLeftGlyph == currentLeftGlyph &&
      requestedRightGlyph == currentRightGlyph) {
    return;
  }

  debugDisplayForceRefresh = false;

  currentLeftGlyph = requestedLeftGlyph;
  currentRightGlyph = requestedRightGlyph;

  writeRawGlyph(DISP_TENS_PINS, currentLeftGlyph);
  writeRawGlyph(DISP_ONES_PINS, currentRightGlyph);
}

static uint32_t lastPrint = 0;
static uint32_t lastRobotSendMs = 0;

#define TEENSY_READY_PIN D3   // ESP32 D3 / GPIO3, driven by Teensy GPIO27



// Robot IMU body axes expressed in world coordinates.
float r_xx = 0.0f, r_xy = 0.0f, r_xz = 0.0f;
float r_yx = 0.0f, r_yy = 0.0f, r_yz = 0.0f;
float r_zx = 0.0f, r_zy = 0.0f, r_zz = 0.0f;

#pragma pack(push, 1)
struct Packet {
  uint8_t  starter[4];
  uint16_t seq;

  // IMU body axes expressed in world coordinates.
  float x_axis_x;
  float x_axis_y;
  float x_axis_z;

  float y_axis_x;
  float y_axis_y;
  float y_axis_z;

  float z_axis_x;
  float z_axis_y;
  float z_axis_z;

  uint8_t crc;
};

struct CachedDataset {
  bool valid;
  uint32_t lastMs;

  float xx;
  float xy;
  float xz;

  float yx;
  float yy;
  float yz;

  float zx;
  float zy;
  float zz;
};

CachedDataset cachedDataset[4] = {
  {false, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {false, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {false, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {false, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
};

void storeDataset(uint8_t datasetId,
                  float xx, float xy, float xz,
                  float yx, float yy, float yz,
                  float zx, float zy, float zz) {
  if (datasetId > 3) {
    return;
  }

  cachedDataset[datasetId].valid = true;
  cachedDataset[datasetId].lastMs = millis();

  cachedDataset[datasetId].xx = xx;
  cachedDataset[datasetId].xy = xy;
  cachedDataset[datasetId].xz = xz;

  cachedDataset[datasetId].yx = yx;
  cachedDataset[datasetId].yy = yy;
  cachedDataset[datasetId].yz = yz;

  cachedDataset[datasetId].zx = zx;
  cachedDataset[datasetId].zy = zy;
  cachedDataset[datasetId].zz = zz;
}

#pragma pack(pop)

uint8_t crcXor(const uint8_t* data, size_t n) {
  uint8_t c = 0;
  for (size_t i = 0; i < n; i++) c ^= data[i];
  return c;
}

uint32_t lastRx1 = 0;
uint32_t lastRx2 = 0;
uint32_t lastRx3 = 0;

uint32_t count1 = 0;
uint32_t count2 = 0;
uint32_t count3 = 0;

static uint32_t lastUdpDebug = 0;

// Sends "0000 ..." / "1111 ..." / "2222 ..." / "3333 ..." to Teensy.
// Format:
// DDDD Xx Xy Xz Yx Yy Yz Zx Zy Zz
void sendDatasetLineToTeensy(uint8_t datasetId,
                             float xx, float xy, float xz,
                             float yx, float yy, float yz,
                             float zx, float zy, float zz) {
  char d = '0' + datasetId;  // 0,1,2,3...
  char line[192];

  int n = snprintf(line, sizeof(line),
                   "%c%c%c%c %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n",
                   d, d, d, d,
                   xx, xy, xz,
                   yx, yy, yz,
                   zx, zy, zz);

  Serial0.write((const uint8_t*)line, n);  // UART -> Teensy
}

void bodyAxesFromQuat(float qw, float qx, float qy, float qz,
                      float &xx, float &xy, float &xz,
                      float &yx, float &yy, float &yz,
                      float &zx, float &zy, float &zz) {
  // Rotation matrix columns = IMU body axes expressed in world coordinates.
  // X body axis in world frame
  xx = 1.0f - 2.0f * (qy * qy + qz * qz);
  xy = 2.0f * (qx * qy + qw * qz);
  xz = 2.0f * (qx * qz - qw * qy);

  // Y body axis in world frame
  yx = 2.0f * (qx * qy - qw * qz);
  yy = 1.0f - 2.0f * (qx * qx + qz * qz);
  yz = 2.0f * (qy * qz + qw * qx);

  // Z body axis in world frame
  zx = 2.0f * (qx * qz + qw * qy);
  zy = 2.0f * (qy * qz - qw * qx);
  zz = 1.0f - 2.0f * (qx * qx + qy * qy);
}

void setReports() {
  Serial.println("Setting reports...");
  if (imu.enableRotationVector(50)) {
    delay(100);
    Serial.println("Rotation Vector enabled @ 50 Hz");
  } else {
    delay(100);
    Serial.println("FAILED to enable Rotation Vector");
  }
  delay(100);
}

bool wifiInit() {
  WiFi.mode(WIFI_AP);
  bool ok = WiFi.softAP(AP_SSID, AP_PASS);
  if (!ok) {
    Serial.println("Failed to start AP");
    return false;
  }

  udp.begin(LISTEN_PORT);

  Serial.print("Robot AP IP: ");
  Serial.println(WiFi.softAPIP());
  Serial.println("UDP listening...");
  return true;
}

// Forward user-side packet -> Teensy dataset 1111 / 2222 / 3333
void receiveUdpAndCacheUserData() {
  while (true) {
    int packetSize = udp.parsePacket();
    if (packetSize <= 0) break;

    if (packetSize != (int)sizeof(Packet)) {
      while (udp.available()) udp.read();
      continue;
    }

    Packet p;
    int n = udp.read((uint8_t*)&p, sizeof(p));
    if (n != (int)sizeof(p)) continue;

    uint8_t calc = crcXor((uint8_t*)&p, sizeof(Packet) - 1);
    if (calc != p.crc) continue;

    uint8_t s0 = p.starter[0];
    uint8_t s1 = p.starter[1];
    uint8_t s2 = p.starter[2];
    uint8_t s3 = p.starter[3];

    uint32_t now = millis();

    if (!(s0 == s1 && s1 == s2 && s2 == s3)) continue;
    if (!(s0 == 1 || s0 == 2 || s0 == 3)) continue;

    if (s0 == 1) {
      lastRx1 = now;
      count1++;
    }
    else if (s0 == 2) {
      lastRx2 = now;
      count2++;
    }
    else if (s0 == 3) {
      lastRx3 = now;
      count3++;
    }

    storeDataset(s0,
             p.x_axis_x, p.x_axis_y, p.x_axis_z,
             p.y_axis_x, p.y_axis_y, p.y_axis_z,
             p.z_axis_x, p.z_axis_y, p.z_axis_z);
  }
}

// Read local robot IMU -> Teensy dataset 0000
void updateRobotImuAndCache() {
  if (!imu_ok) return;

  if (imu.wasReset()) {
    Serial.println("IMU was reset; re-enabling reports...");
    setReports();
  }

  if (imu.getSensorEvent() &&
      imu.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {

    float qw = imu.getQuatReal();
    float qx = imu.getQuatI();
    float qy = imu.getQuatJ();
    float qz = imu.getQuatK();

    bodyAxesFromQuat(qw, qx, qy, qz,
                     r_xx, r_xy, r_xz,
                     r_yx, r_yy, r_yz,
                     r_zx, r_zy, r_zz);

    uint32_t now = millis();
    if (now - lastRobotSendMs >= 20) { // 50 Hz
      lastRobotSendMs = now;

      // Robot-side IMU/reference
      storeDataset(0,
                   r_xx, r_xy, r_xz,
                   r_yx, r_yy, r_yz,
                   r_zx, r_zy, r_zz);
    }

    if (now - lastPrint >= 200) {
      lastPrint = now;
      Serial.print("robot imu | X=");
      Serial.print(r_xx, 3); Serial.print(",");
      Serial.print(r_xy, 3); Serial.print(",");
      Serial.print(r_xz, 3);
      Serial.print(" | Y=");
      Serial.print(r_yx, 3); Serial.print(",");
      Serial.print(r_yy, 3); Serial.print(",");
      Serial.print(r_yz, 3);
      Serial.print(" | Z=");
      Serial.print(r_zx, 3); Serial.print(",");
      Serial.print(r_zy, 3); Serial.print(",");
      Serial.println(r_zz, 3);
    }
  }
}

void updateLeds() {
  static uint32_t lastBlink = 0;
  static bool ledState = false;

  uint32_t now = millis();

  bool imuError = !imu_ok;
  bool userConnected = (now - lastRx1 < 1000) || 
                       (now - lastRx2 < 1000) || 
                       (now - lastRx3 < 1000);

  if (imuError) {
    if (now - lastBlink >= 300) {
      lastBlink = now;
      ledState = !ledState;
      digitalWrite(ledRed, !ledState);
      digitalWrite(ledGreen, HIGH);
      digitalWrite(ledBlue, HIGH);
    }
  }
  else if (!userConnected) {
    if (now - lastBlink >= 300) {
      lastBlink = now;
      ledState = !ledState;
      digitalWrite(ledRed, HIGH);
      digitalWrite(ledGreen, HIGH);
      digitalWrite(ledBlue, !ledState);
    }
  }
  else {
    digitalWrite(ledRed, HIGH);
    digitalWrite(ledGreen, LOW);
    digitalWrite(ledBlue, HIGH);
  }
}

void sendCachedDataToTeensyWhenReady() {
  static uint32_t lastSendMs = 0;

  bool readyNow = digitalRead(TEENSY_READY_PIN);

  if (!readyNow) {
    return;
  }

  uint32_t now = millis();

  if (now - lastSendMs < 5) {
    return;
  }

  lastSendMs = now;

  for (uint8_t id = 0; id < 4; id++) {
    if (!cachedDataset[id].valid) {
      continue;
    }

    if (id != 0 && now - cachedDataset[id].lastMs > 150) {
      continue;
    }

    sendDatasetLineToTeensy(id,
                            cachedDataset[id].xx,
                            cachedDataset[id].xy,
                            cachedDataset[id].xz,
                            cachedDataset[id].yx,
                            cachedDataset[id].yy,
                            cachedDataset[id].yz,
                            cachedDataset[id].zx,
                            cachedDataset[id].zy,
                            cachedDataset[id].zz);
  }

  Serial0.flush();
}

void updateDebugStatusCode() {
  uint32_t now = millis();

  bool userConnected =
    (now - lastRx1 < 1000) ||
    (now - lastRx2 < 1000) ||
    (now - lastRx3 < 1000);

  bool allUserImusFresh =
    (now - lastRx1 < 500) &&
    (now - lastRx2 < 500) &&
    (now - lastRx3 < 500);

  bool teensyReady = digitalRead(TEENSY_READY_PIN);

  if (!wifi_ok) {
    requestDebugFault(FAULT_COMM);
    return;
  }

  if (!imu_ok) {
    requestDebugFault(FAULT_IMU);
    return;
  }

  if (!userConnected) {
    requestDebugCode(DBG_WAIT_USER);      // 01
    return;
  }

  if (!teensyReady) {
    requestDebugCode(DBG_WAIT_TEENSY);    // 03
    return;
  }

  if (!allUserImusFresh) {
    requestDebugFault(FAULT_IMU);         // E2
    return;
  }

  requestDebugCode(DBG_TELEOP_ACTIVE);    // 07
}

void setup() {
  pinMode(ledRed, OUTPUT);
  pinMode(ledGreen, OUTPUT);
  pinMode(ledBlue, OUTPUT);
  pinMode(TEENSY_READY_PIN, INPUT);

  initDebugDisplay();
  requestDebugCode(DBG_BOOT);   // 00
  updateDebugDisplay();

  delay(1000);
  Serial.begin(2000000);  // USB serial for debug
  delay(500);
  Serial0.begin(2000000);   // UART to Teensy

  Serial.println("Robot-side ESP32 starting...");
  // IMU init: same pattern as your working user-side setup
  pinMode(BNO08X_RST, OUTPUT);
  digitalWrite(BNO08X_RST, LOW);
  delay(20);
  digitalWrite(BNO08X_RST, HIGH);
  delay(200);

  SPI.begin();

  imu_ok = imu.beginSPI(BNO08X_CS, BNO08X_INT, BNO08X_RST);
  if (!imu_ok) {
    Serial.println("BNO08x not detected. rebooting IMU.");
  } else {
    Serial.println("BNO08x detected!");
    setReports();
  }

  wifi_ok = wifiInit();
  updateLeds();

  Serial.println("BOOT COMPLETE");
}

void loop() {
  receiveUdpAndCacheUserData();
  updateRobotImuAndCache();
  sendCachedDataToTeensyWhenReady();
  updateLeds();
  uint32_t now = millis();

if (now - lastUdpDebug >= 200) {
  lastUdpDebug = now;

  Serial.print("[UDP] ");
  Serial.print("1:");
  Serial.print(now - lastRx1 < 500 ? "OK " : "NO ");
  Serial.print(count1);

  Serial.print(" | 2:");
  Serial.print(now - lastRx2 < 500 ? "OK " : "NO ");
  Serial.print(count2);

  Serial.print(" | 3:");
  Serial.print(now - lastRx3 < 500 ? "OK " : "NO ");
  Serial.print(count3);

  Serial.print(" | robotIMU:");
  Serial.println(imu_ok ? "OK" : "NO");
}
  updateDebugStatusCode();
  updateDebugDisplay();
}
