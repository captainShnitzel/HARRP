#include <Arduino.h>
#include <SPI.h>
#include "BNO08x_MultiSPI.h"
#include <WiFi.h>
#include <WiFiUdp.h>

// =========================
// IMU objects
// =========================
BNO08x imu1;
BNO08x imu2;
BNO08x imu3;

// =========================
// Pin definitions
// =========================
#define BNO08X_CS1   10
#define BNO08X_CS2   9
#define BNO08X_CS3   8
#define BNO08X_INT1  7
#define BNO08X_INT2  6
#define BNO08X_INT3  3
#define BNO08X_RST   4

#define ledRed       14
#define ledGreen     15
#define ledBlue      16

static constexpr int NUM_FINGERS = 6;

static constexpr int FLEX_ADC_PIN = A3;

static constexpr int MUX_S0 = A4;
static constexpr int MUX_S1 = A5;
static constexpr int MUX_S2 = A6;
static constexpr int MUX_S3 = A7;

// MUX channels for the 6 flex sensors
static constexpr uint8_t FLEX_MUX_CH[NUM_FINGERS] = {
  0,  // Thumb flex
  1,  // Thumb rotation
  2,  // Index
  3,  // Middle
  4,  // Ring
  5   // Pinkie
};

// Measured calibration voltages
static constexpr float FLEX_OPEN_V[NUM_FINGERS] = {
  1.90f,  // Thumb flex
  1.65f,  // Thumb rotation
  2.00f,  // Index
  2.00f,  // Middle
  2.00f,  // Ring
  2.00f   // Pinkie
};

static constexpr float FLEX_CLOSED_V[NUM_FINGERS] = {
  1.25f,  // Thumb flex
  1.30f,  // Thumb rotation
  1.16f,  // Index
  1.14f,  // Middle
  1.16f,  // Ring
  1.10f   // Pinkie
};

struct Kalman1D {
  float x;
  float p;
  float q;
  float r;
  bool initialized;

  void begin(float initialValue, float processNoise, float measurementNoise) {
    x = initialValue;
    p = 1.0f;
    q = processNoise;
    r = measurementNoise;
    initialized = true;
  }

  float update(float measurement) {
    if (!initialized) {
      begin(measurement, 0.00005f, 0.0008f);
      return measurement;
    }

    p = p + q;

    float k = p / (p + r);
    x = x + k * (measurement - x);
    p = (1.0f - k) * p;

    return x;
  }
};

Kalman1D flexKalman[NUM_FINGERS];

float flexVoltageRaw[NUM_FINGERS];
float flexVoltageFiltered[NUM_FINGERS];
float flexCommand[NUM_FINGERS];

void setupMuxPins() {
  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(MUX_S3, OUTPUT);

  digitalWrite(MUX_S0, LOW);
  digitalWrite(MUX_S1, LOW);
  digitalWrite(MUX_S2, LOW);
  digitalWrite(MUX_S3, LOW);
}

void selectMuxChannel(uint8_t channel) {
  digitalWrite(MUX_S0, (channel & 0x01) ? HIGH : LOW);
  digitalWrite(MUX_S1, (channel & 0x02) ? HIGH : LOW);
  digitalWrite(MUX_S2, (channel & 0x04) ? HIGH : LOW);
  digitalWrite(MUX_S3, (channel & 0x08) ? HIGH : LOW);

  delayMicroseconds(20);
}

float readMuxVoltage(uint8_t channel) {
  selectMuxChannel(channel);

  const int samples = 8;
  uint32_t totalMv = 0;

  for (int i = 0; i < samples; i++) {
    totalMv += analogReadMilliVolts(FLEX_ADC_PIN);
    delayMicroseconds(100);
  }

  float mv = (float)totalMv / (float)samples;
  return mv / 1000.0f;
}

float voltageToFlexCommand(float voltage, float openVoltage, float closedVoltage) {
  float denominator = openVoltage - closedVoltage;

  if (fabsf(denominator) < 0.05f) {
    return 0.0f;
  }

  float command = (openVoltage - voltage) / denominator;

  if (command < 0.0f) command = 0.0f;
  if (command > 1.0f) command = 1.0f;

  return command;
}

float applyDeadband(float value, float deadband) {
  if (value < deadband) {
    return 0.0f;
  }

  if (value > 1.0f - deadband) {
    return 1.0f;
  }

  return value;
}

void readFlexSensors() {
  for (int i = 0; i < NUM_FINGERS; i++) {
    float rawV = readMuxVoltage(FLEX_MUX_CH[i]);
    float filtV = flexKalman[i].update(rawV);
    float cmd = voltageToFlexCommand(filtV, FLEX_OPEN_V[i], FLEX_CLOSED_V[i]);

    cmd = applyDeadband(cmd, 0.03f);

    flexVoltageRaw[i] = rawV;
    flexVoltageFiltered[i] = filtV;
    flexCommand[i] = cmd;
  }
}

// =========================
// Wi-Fi config
// =========================
const char* WIFI_SSID = "robot";
const char* WIFI_PASS = "12345678";
IPAddress ROBOT_IP(192, 168, 4, 1);
const uint16_t ROBOT_PORT = 5005;

WiFiUDP udp;

// =========================
// Status flags
// =========================
bool imu_ok1 = false;
bool imu_ok2 = false;
bool imu_ok3 = false;
bool wifi_ok = false;

// =========================
// Timing
// =========================
static uint32_t lastPrint = 0;
static uint32_t lastFlash = 0;
static uint32_t lastSendMs1 = 0;
static uint32_t lastSendMs2 = 0;
static uint32_t lastSendMs3 = 0;

static uint32_t lastUsbPrint = 0;

const uint32_t IMU_SPI_SPEED = 3000000;

// Separate sequence counters per IMU
static uint16_t seq1 = 0;
static uint16_t seq2 = 0;
static uint16_t seq3 = 0;


// =========================
// Math helpers
// =========================
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

// =========================
// Packet definition
// starter[4] = 1,1,1,1 for IMU1
// starter[4] = 2,2,2,2 for IMU2
// later 3,3,3,3 for IMU3
// =========================
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
#pragma pack(pop)

uint8_t crcXor(const uint8_t* data, size_t n) {
  uint8_t c = 0;
  for (size_t i = 0; i < n; i++) {
    c ^= data[i];
  }
  return c;
}

struct Quat {
  float w, x, y, z;
  bool valid;
};

Quat q1 = {0,0,0,0,false};
Quat q2 = {0,0,0,0,false};
Quat q3 = {0,0,0,0,false};

// =========================
// IMU setup helper
// =========================
bool setReports(BNO08x &imu, const char *name) {
  Serial.print("Setting reports for ");
  Serial.println(name);

  for (int attempt = 1; attempt <= 5; attempt++) {
    Serial.print(name);
    Serial.print(": report attempt ");
    Serial.println(attempt);

    imu.getSensorEvent();
    delay(100);

    bool ok = imu.enableRotationVector(200);
    delay(200);

    if (ok) {
      Serial.print(name);
      Serial.println(": Rotation Vector enabled @ 200 Hz");
      return true;
    }
  }

  Serial.print(name);
  Serial.println(": FAILED to enable Rotation Vector");
  return false;
}

// =========================
// Wi-Fi setup
// =========================
bool wifiInit(uint32_t timeoutMs = 8000) {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.print("WiFi connecting");
  uint32_t start = millis();

  while (WiFi.status() != WL_CONNECTED && (millis() - start) < timeoutMs) {
    delay(250);
    Serial.print(".");
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nWiFi timeout, continuing without network");
    return false;
  }

  Serial.println("\nWiFi connected.");
  Serial.print("User IP: ");
  Serial.println(WiFi.localIP());

  udp.begin(ROBOT_PORT);
  Serial.println("UDP ready.");
  return true;
}

// =========================
// Generic sender for each IMU
// starterValue:
//   1 -> packet begins 1,1,1,1
//   2 -> packet begins 2,2,2,2
// =========================
void sendPacket(uint8_t starterValue,
                uint16_t &seq,
                uint32_t &lastSendMs,
                float xx, float xy, float xz,
                float yx, float yy, float yz,
                float zx, float zy, float zz) {
  if (!wifi_ok) return;

  uint32_t now = millis();
  if (now - lastSendMs < 1) return;  // limit to ~1000 Hz
  lastSendMs = now;

  Packet p;
  p.starter[0] = starterValue;
  p.starter[1] = starterValue;
  p.starter[2] = starterValue;
  p.starter[3] = starterValue;
  p.seq = seq++;
  p.x_axis_x = xx;
  p.x_axis_y = xy;
  p.x_axis_z = xz;
  p.y_axis_x = yx;
  p.y_axis_y = yy;
  p.y_axis_z = yz;
  p.z_axis_x = zx;
  p.z_axis_y = zy;
  p.z_axis_z = zz;
  p.crc = 0;
  p.crc = crcXor(reinterpret_cast<uint8_t*>(&p), sizeof(Packet) - 1);

  udp.beginPacket(ROBOT_IP, ROBOT_PORT);
  udp.write(reinterpret_cast<uint8_t*>(&p), sizeof(Packet));
  udp.endPacket();

}

// =========================
// Read one IMU and send one packet
// =========================
void handleImu(BNO08x &imu,
               bool imu_ok,
               const char *name,
               uint8_t starterValue,
               uint16_t &seq,
               uint32_t &lastSendMs) {
  if (!imu_ok) return;

  if (imu.wasReset()) {
  Serial.print(name);
  Serial.println(" was reset; re-enabling reports...");

  if (!setReports(imu, name)) {
    Serial.print(name);
    Serial.println(" report re-enable failed; disabling this IMU.");
    return;
  }
}

  if (imu.getSensorEvent() &&
      imu.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {

    float qw = imu.getQuatReal();
    float qx = imu.getQuatI();
    float qy = imu.getQuatJ();
    float qz = imu.getQuatK();

    float xx, xy, xz;
    float yx, yy, yz;
    float zx, zy, zz;
    bodyAxesFromQuat(qw, qx, qy, qz,
                     xx, xy, xz,
                     yx, yy, yz,
                     zx, zy, zz);

    if (starterValue == 1) {
        q1 = {qw, qx, qy, qz, true};
      }
      else if (starterValue == 2) {
        q2 = {qw, qx, qy, qz, true};
      }
      else if (starterValue == 3) {
        q3 = {qw, qx, qy, qz, true};
      }

    sendPacket(starterValue, seq, lastSendMs,
               xx, xy, xz,
               yx, yy, yz,
               zx, zy, zz);
  }
}

void printFlexDebug() {
  Serial.println("Finger readings:");

  for (int i = 0; i < NUM_FINGERS; i++) {
    Serial.print(i);
    Serial.print(" raw=");
    Serial.print(flexVoltageRaw[i], 3);
    Serial.print("V filtered=");
    Serial.print(flexVoltageFiltered[i], 3);
    Serial.print("V command=");
    Serial.println(flexCommand[i], 3);
  }

  Serial.println();
}
// =========================
// Arduino setup
// =========================
void setup() {
  pinMode(ledRed, OUTPUT);
  pinMode(ledGreen, OUTPUT);
  pinMode(ledBlue, OUTPUT);

  delay(2000);
  Serial.begin(2000000);
  delay(1000);

  Serial.println("BOOT START");

  SPI.begin(13, 12, 11);

pinMode(BNO08X_CS1, OUTPUT);
pinMode(BNO08X_CS2, OUTPUT);
pinMode(BNO08X_CS3, OUTPUT);
pinMode(BNO08X_RST, OUTPUT);
digitalWrite(BNO08X_RST, LOW);
delay(20);
digitalWrite(BNO08X_RST, HIGH);
delay(200);
digitalWrite(BNO08X_CS1, HIGH);
digitalWrite(BNO08X_CS2, HIGH);
digitalWrite(BNO08X_CS3, HIGH);

  delay(500);

digitalWrite(BNO08X_CS1, HIGH);
digitalWrite(BNO08X_CS2, HIGH);
digitalWrite(BNO08X_CS3, HIGH);
delay(200);

imu_ok1 = imu1.beginSPI(BNO08X_CS1, BNO08X_INT1, BNO08X_RST, IMU_SPI_SPEED);
delay(300);

imu_ok2 = imu2.beginSPI(BNO08X_CS2, BNO08X_INT2, BNO08X_RST, IMU_SPI_SPEED);
delay(300);

imu_ok3 = imu3.beginSPI(BNO08X_CS3, BNO08X_INT3, BNO08X_RST, IMU_SPI_SPEED);
delay(500);

if (imu_ok1) {
  Serial.println("IMU1 detected.");
} else {
  Serial.println("IMU1 not detected. Continuing without IMU1.");
}

if (imu_ok2) {
  Serial.println("IMU2 detected.");
} else {
  Serial.println("IMU2 not detected. Continuing without IMU2.");
}

if (imu_ok3) {
  Serial.println("IMU3 detected.");
} else {
  Serial.println("IMU3 not detected. Continuing without IMU3.");
}

delay(500);

imu1.getSensorEvent();
delay(150);
imu2.getSensorEvent();
delay(150);
imu3.getSensorEvent();
delay(300);

if (imu_ok1) {
  imu_ok1 = setReports(imu1, "IMU1");
}

delay(300);

if (imu_ok2) {
  imu_ok2 = setReports(imu2, "IMU2");
}

delay(300);

if (imu_ok3) {
  imu_ok3 = setReports(imu3, "IMU3");
}

delay(500);

  wifi_ok = wifiInit();

  setupMuxPins();

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  for (int i = 0; i < NUM_FINGERS; i++) {
    flexKalman[i].begin(FLEX_OPEN_V[i], 0.00005f, 0.0008f);
    flexVoltageRaw[i] = FLEX_OPEN_V[i];
    flexVoltageFiltered[i] = FLEX_OPEN_V[i];
    flexCommand[i] = 0.0f;
  }

  Serial.println("BOOT COMPLETE");
}

// =========================
// Arduino loop
// =========================
void loop() {
  uint32_t now = millis();

  handleImu(imu1, imu_ok1, "IMU1", 1, seq1, lastSendMs1);
  handleImu(imu2, imu_ok2, "IMU2", 2, seq2, lastSendMs2);
  handleImu(imu3, imu_ok3, "IMU3", 3, seq3, lastSendMs3);

  // ===== WIFI RECONNECT MANAGER =====
  static uint32_t lastWifiCheck = 0;
  static uint32_t lastReconnectAttempt = 0;

  bool wifiConnected = (WiFi.status() == WL_CONNECTED);
  wifi_ok = wifiConnected;

  if (now - lastWifiCheck >= 500) {
    lastWifiCheck = now;

    if (!wifiConnected && now - lastReconnectAttempt >= 2000) {
      lastReconnectAttempt = now;

      Serial.println("[WiFi] disconnected, reconnecting...");
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASS);
    }
  }

  // ===== LED STATE MACHINE =====
  static uint32_t lastBlink = 0;
  static bool ledState = false;

  bool imuError = !(imu_ok1 && imu_ok2 && imu_ok3);

  if (imuError) {
    if (now - lastBlink >= 300) {
      lastBlink = now;
      ledState = !ledState;

      digitalWrite(ledRed, !ledState);
      digitalWrite(ledGreen, HIGH);
      digitalWrite(ledBlue, HIGH);
    }
  }
  else if (!wifiConnected) {
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

  readFlexSensors();

  // ===== UNIFIED USB DEBUG =====
  if (now - lastUsbPrint >= 50) {
    lastUsbPrint = now;

    Serial.print("[IMU] ");

    Serial.print("1:");
    Serial.print(q1.valid ? "OK " : "NO ");

    Serial.print("| 2:");
    Serial.print(q2.valid ? "OK " : "NO ");

    Serial.print("| 3:");
    Serial.print(q3.valid ? "OK " : "NO ");

    Serial.print(" || ");

    if (q1.valid) {
      Serial.print("q1(");
      Serial.print(q1.w,3); Serial.print(",");
      Serial.print(q1.x,3); Serial.print(",");
      Serial.print(q1.y,3); Serial.print(",");
      Serial.print(q1.z,3); Serial.print(") ");
    }

    if (q2.valid) {
      Serial.print("q2(");
      Serial.print(q2.w,3); Serial.print(",");
      Serial.print(q2.x,3); Serial.print(",");
      Serial.print(q2.y,3); Serial.print(",");
      Serial.print(q2.z,3); Serial.print(") ");
    }

    if (q3.valid) {
      Serial.print("q3(");
      Serial.print(q3.w,3); Serial.print(",");
      Serial.print(q3.x,3); Serial.print(",");
      Serial.print(q3.y,3); Serial.print(",");
      Serial.print(q3.z,3); Serial.print(") ");
    }

    Serial.println();
    printFlexDebug();
  }

  //delay(1);
}