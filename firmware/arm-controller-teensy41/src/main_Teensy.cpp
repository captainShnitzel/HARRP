//Teensy 4.1 main movement computation 1.21
#include <Arduino.h>
#include <math.h>
#include <Dynamixel2Arduino.h>

// ============================================================
// ROBOT KINEMATICS CONTROLLER:
//
// Input UART line format from robot communication module:
//   0000 Xx Xy Xz Yx Yy Yz Zx Zy Zz   robot stationary reference IMU
//   1111 Xx Xy Xz Yx Yy Yz Zx Zy Zz   user upper-arm IMU
//   2222 Xx Xy Xz Yx Yy Yz Zx Zy Zz   user lower-arm IMU
//
// Meanings used here:
//   User upper-arm IMU:
//     +X = along user's upper arm length
//     +Z = up/down direction on user arm frame
//     +Y = left/right direction on user arm frame
//
//   Robot onboard IMU / reference frame:
//     +X = along robot upper arm length
//     +Y = completes right-hand frame
//     +Z = motor 1 rotation axis
//
// Shoulder chain assumed:
//   Motor 1 / ID 1: yaw foreward - backward, first in hierarchy, axis +Z_ref
//   Motor 2 / ID 2: pitch up-down, second in hierarchy, axis +Y after motor 1
//   Motor 3 / ID 3: roll/twist, third in hierarchy, axis +X along arm link
//
// Rotation model for DELTAS from startup pose:
//   R_delta = Rz(q1) * Ry(q2) * Rx(q3)
//
// Command model:
//   motor_target_raw_deg = motor_start_raw_deg + map.sign * map.gear * q_delta_deg
//
// 
// ============================================================

#define DEBUG_SERIAL Serial
#define REF_SERIAL   Serial1
#define DXL_SERIAL   Serial2

#define N 5  // 3–7 is a good range for runing avarege on noisy IMU data. Too high adds latency.

#define DXL_DIR_PIN 28
#define ESP_READY_PIN 27

Dynamixel2Arduino dxl2(DXL_SERIAL, DXL_DIR_PIN);  // XH motors, Protocol 2.0
Dynamixel2Arduino dxl1(DXL_SERIAL, DXL_DIR_PIN);  // MX motors, Protocol 1.0


static constexpr uint32_t DBG_BAUD = 2000000;
static constexpr uint32_t REF_BAUD = 2000000;
static constexpr uint32_t DXL_BAUD = 3000000;

static constexpr bool DRIVE_MOTORS = true;


static constexpr uint8_t DXL_ID_Q1 = 1;
static constexpr uint8_t DXL_ID_Q2 = 2;
static constexpr uint8_t DXL_ID_Q3 = 3;
static constexpr uint8_t DXL_ID_Q4 = 4;
static constexpr uint8_t DXL_ID_Q5 = 5;
static constexpr uint8_t DXL_ID_Q6 = 6;
static constexpr uint8_t DXL_ID_Q7 = 7;

// User told us motors 1 and 2 are reversed and 4:1 reduced.
// Motor 3 is 1:1. Change Q3 sign if roll is reversed during testing.
struct MotorMap { float sign; float gear; };
static constexpr MotorMap MAP_Q1 = {-1.0f, 4.0f};
static constexpr MotorMap MAP_Q2 = {-1.0f, 4.0f};
static constexpr MotorMap MAP_Q3 = {+1.0f, 1.0f};
static constexpr MotorMap MAP_Q4 = {1.0f, 3.0f};
static constexpr MotorMap MAP_Q5 = {1.0f, 1.0f};
static constexpr MotorMap MAP_Q6 = {-1.0f, 4.0f};
static constexpr MotorMap MAP_Q7 = {-1.0f, 4.0f};

static constexpr float PI_F = 3.14159265358979323846f;
static constexpr float DEG2RAD = PI_F / 180.0f;
static constexpr float RAD2DEG = 180.0f / PI_F;

// Relative movement safety limits from startup pose.
// Q1: no full rotation. Kept conservative for first test.
// Q2: allowed upward range; if direction is opposite, flip Q2 math/sign after test.
// Q3: user requested +/-45 degrees.
static constexpr float Q1_DELTA_MIN_DEG = -500.0f;
static constexpr float Q1_DELTA_MAX_DEG = +500.0f;
static constexpr float Q2_DELTA_MIN_DEG = -500.0f;
static constexpr float Q2_DELTA_MAX_DEG = +500.0f;
static constexpr float Q3_DELTA_MIN_DEG = -500.0f;
static constexpr float Q3_DELTA_MAX_DEG = +500.0f;
static constexpr float Q4_DELTA_MIN_DEG = -10.0f;
static constexpr float Q4_DELTA_MAX_DEG = +80.0f;
static constexpr float Q5_DELTA_MIN_DEG = -90.0f;
static constexpr float Q5_DELTA_MAX_DEG = +90.0f;
static constexpr float Q6_DELTA_MIN_DEG = -70.0f;
static constexpr float Q6_DELTA_MAX_DEG = +70.0f;
static constexpr float Q7_DELTA_MIN_DEG = -70.0f;
static constexpr float Q7_DELTA_MAX_DEG = +70.0f;

// Per-loop slew rate. REF loop is 10 ms, so these are intentionally gentle.
static constexpr float MAX_STEP_Q1_DEG = 0.7f;
static constexpr float MAX_STEP_Q2_DEG = 0.7f;
static constexpr float MAX_STEP_Q3_DEG = 0.7f;
static constexpr float MAX_STEP_Q4_DEG = 0.7f;
static constexpr float MAX_STEP_Q5_DEG = 0.7f;
static constexpr float MAX_STEP_Q6_DEG = 0.7f;
static constexpr float MAX_STEP_Q7_DEG = 0.7f;

// If your physical pitch sign is opposite after the first test, flip this.
static constexpr float SHOULDER_Q1_MATH_SIGN = +1.0f;
static constexpr float SHOULDER_Q2_MATH_SIGN = +1.0f;
static constexpr float SHOULDER_Q3_MATH_SIGN = +1.0f;

static constexpr float ELBOW_Q4_MATH_SIGN = +1.0f;
static constexpr float ELBOW_Q5_MATH_SIGN = +1.0f;
static bool elbowFilterInit = false;

static constexpr float WRIST_Q6_MATH_SIGN = -1.0f;
static constexpr float WRIST_Q7_MATH_SIGN = -1.0f;


static float q5ForWristReference = 0.0f;
static float wristQ6Filt = 0.0f;
static float wristQ7Filt = 0.0f;
static bool wristFilterInit = false;

static const float WRIST_LPF_ALPHA = 0.15f; // lower = smoother, higher = faster


static float shoulderQ1Filt = 0.0f;
static float shoulderQ2Filt = 0.0f;
static float shoulderQ3Filt = 0.0f;
static bool shoulderFilterInit = false;


static const float SHOULDER_LPF_ALPHA = 0.08f; // start here

struct Vec3 { float x, y, z; };
struct Mat3 { float m[3][3]; };

struct FrameData {
  Mat3 R_world;     // columns are IMU body axes expressed in BNO/world frame
  bool valid = false;
  uint32_t lastMs = 0;
};

static FrameData robotRefFrame;  // 0000
static FrameData userUpperFrame; // 1111
static FrameData userForearmFrame; // 2222
static FrameData userPalmFrame; // 3333

static bool calibrated = false;
static Mat3 robotRefZero_world;
static Mat3 userUpperZero_ref;
static Mat3 userUpperZero_ref_T;
static Mat3 userForearmZero_ref;
static Mat3 userForearmZero_ref_T;
static Mat3 userPalmZero_ref;
static Mat3 userPalmZero_ref_T;
static float motorStartRawDeg[8] = {0}; // indexed by ID

struct ShoulderDelta {
  float q1 = 0.0f; // radians
  float q2 = 0.0f;
  float q3 = 0.0f;
};

struct ElbowDelta {
  float q4 = 0.0f; // radians
  float q5 = 0.0f;
};

struct WristDelta {
  float q6 = 0.0f; // radians
  float q7 = 0.0f;
};

static ShoulderDelta qCmd;    // slewed command deltas
static ShoulderDelta qTarget; // instant target deltas

static ElbowDelta qElbow;
static ElbowDelta qElbowCmd;
static ElbowDelta qElbowTarget;
static WristDelta qWristCmd;
static WristDelta qWristTarget;


static uint32_t lastControlMs = 0;
static uint32_t lastDebugMs = 0;

// UART input-chain diagnostics.
static uint32_t rxGoodLines = 0;
static uint32_t rxBadFormat = 0;
static uint32_t rxBadTag = 0;
static uint32_t rxOverflow = 0;
static uint32_t rxResyncDrops = 0;
static uint32_t rxPalm3333 = 0;
// Extra Serial1 RX buffering for bursts from the Nano ESP32.
// Teensy HardwareSerial supports addMemoryForRead(). The default RX buffer is too small
// for several long ASCII IMU packets arriving back-to-back.
static uint8_t refSerialRxExtraBuffer[8192];

static constexpr size_t RX_LINE_MAX = 192;
static constexpr uint32_t RX_PROCESS_BUDGET_US = 1200;  // keep motor loop from being starved
static constexpr uint16_t RX_MAX_BYTES_PER_LOOP = 192;

static bool imuDataFresh(uint32_t now);
static void resetMotionState();

static inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static inline float wrapPi(float a) {
  while (a > PI_F) a -= 2.0f * PI_F;
  while (a < -PI_F) a += 2.0f * PI_F;
  return a;
}

static inline float stepToward(float current, float target, float maxStep) {
  float d = target - current;
  if (d > maxStep) d = maxStep;
  if (d < -maxStep) d = -maxStep;
  return current + d;
}

static inline Vec3 makeVec3(float x, float y, float z) { return {x, y, z}; }

static inline float dot(const Vec3& a, const Vec3& b) {
  return a.x*b.x + a.y*b.y + a.z*b.z;
}

static inline float norm(const Vec3& v) { return sqrtf(dot(v, v)); }

static inline Vec3 normalize(const Vec3& v) {
  float n = norm(v);
  if (n < 1e-6f) return makeVec3(1.0f, 0.0f, 0.0f);
  return makeVec3(v.x/n, v.y/n, v.z/n);
}

static inline Mat3 matIdentity() {
  Mat3 I{};
  I.m[0][0] = 1.0f; I.m[1][1] = 1.0f; I.m[2][2] = 1.0f;
  return I;
}

static inline Mat3 matTranspose(const Mat3& A) {
  Mat3 T{};
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) T.m[r][c] = A.m[c][r];
  }
  return T;
}

static inline Mat3 matMul(const Mat3& A, const Mat3& B) {
  Mat3 C{};
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      C.m[r][c] = A.m[r][0]*B.m[0][c] + A.m[r][1]*B.m[1][c] + A.m[r][2]*B.m[2][c];
    }
  }
  return C;
}

static inline Mat3 rotX(float a) {
  Mat3 R = matIdentity();
  float c = cosf(a);
  float s = sinf(a);

  R.m[1][1] = c;
  R.m[1][2] = -s;
  R.m[2][1] = s;
  R.m[2][2] = c;

  return R;
}

static inline Vec3 matCol(const Mat3& A, int c) {
  return makeVec3(A.m[0][c], A.m[1][c], A.m[2][c]);
}

static inline Vec3 matVec(const Mat3& A, const Vec3& v) {
  return makeVec3(
    A.m[0][0]*v.x + A.m[0][1]*v.y + A.m[0][2]*v.z,
    A.m[1][0]*v.x + A.m[1][1]*v.y + A.m[1][2]*v.z,
    A.m[2][0]*v.x + A.m[2][1]*v.y + A.m[2][2]*v.z
  );
}

static inline Vec3 cross(const Vec3& a, const Vec3& b) {
  return makeVec3(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x);
}

static inline Vec3 sub(const Vec3& a, const Vec3& b) { return makeVec3(a.x-b.x, a.y-b.y, a.z-b.z); }
static inline Vec3 scale(const Vec3& v, float k) { return makeVec3(v.x*k, v.y*k, v.z*k); }
static inline Vec3 projectPerp(const Vec3& v, const Vec3& axisUnit) { return sub(v, scale(axisUnit, dot(v, axisUnit))); }
static inline Vec3 add(const Vec3& a, const Vec3& b) { return makeVec3(a.x+b.x, a.y+b.y, a.z+b.z); }

static inline Vec3 rotateAroundAxis(const Vec3& v, const Vec3& axisUnit, float angleRad) {
  float c = cosf(angleRad);
  float ss = sinf(angleRad);
  Vec3 term1 = scale(v, c);
  Vec3 term2 = scale(cross(axisUnit, v), ss);
  Vec3 term3 = scale(axisUnit, dot(axisUnit, v) * (1.0f - c));
  return add(add(term1, term2), term3);
}

// Shortest-arc swing rotation: rotate vector v by the minimal rotation that maps fromUnit -> toUnit.
// Used for roll extraction so that roll is measured around the arm direction only,
// without adding artificial twist from the yaw/pitch Euler decomposition.
static Vec3 swingRotateVector(const Vec3& fromUnit, const Vec3& toUnit, const Vec3& v) {
  float c = clampf(dot(fromUnit, toUnit), -1.0f, 1.0f);
  Vec3 axis = cross(fromUnit, toUnit);
  float ss = norm(axis);

  if (ss < 1e-6f) {
    if (c > 0.0f) return v; // same direction

    // 180-degree case: choose any stable axis perpendicular to fromUnit.
    Vec3 candidate = cross(fromUnit, makeVec3(0.0f, 1.0f, 0.0f));
    if (norm(candidate) < 1e-6f) candidate = cross(fromUnit, makeVec3(0.0f, 0.0f, 1.0f));
    axis = normalize(candidate);
    return rotateAroundAxis(v, axis, PI_F);
  }

  axis = scale(axis, 1.0f / ss);
  float angle = atan2f(ss, c);
  return rotateAroundAxis(v, axis, angle);
}

static inline float signedAngleAroundAxis(const Vec3& fromUnit, const Vec3& toUnit, const Vec3& axisUnit) {
  float sinA = dot(axisUnit, cross(fromUnit, toUnit));
  float cosA = clampf(dot(fromUnit, toUnit), -1.0f, 1.0f);
  return atan2f(sinA, cosA);
}

static inline float unwrapNear(float angle, float reference) {
  while (angle - reference > PI_F) angle -= 2.0f * PI_F;
  while (angle - reference < -PI_F) angle += 2.0f * PI_F;
  return angle;
}

static Mat3 makeMatrixFromColumns(const Vec3& xAxis, const Vec3& yAxis, const Vec3& zAxis) {
  Mat3 R{};
  R.m[0][0] = xAxis.x; R.m[1][0] = xAxis.y; R.m[2][0] = xAxis.z;
  R.m[0][1] = yAxis.x; R.m[1][1] = yAxis.y; R.m[2][1] = yAxis.z;
  R.m[0][2] = zAxis.x; R.m[1][2] = zAxis.y; R.m[2][2] = zAxis.z;
  return R;
}

// User upper-arm IMU mounting correction.
// Symptom fixed here:
//   user rotation about sensor Z was commanding the robot Y-axis,
//   user rotation about sensor Y was commanding the robot Z-axis.
//
// This remaps ONLY the 1111 user IMU body frame before calibration/control:
//   corrected X = raw X
//   corrected Y = raw Z
//   corrected Z = -raw Y
//
// The minus sign keeps the corrected frame right-handed. If the Y/Z axes become
// correct but one direction is reversed, change USER_SWAP_YZ_SIGN to -1.0f.
static constexpr bool USER_SWAP_YZ_CORRECTION = false;
static constexpr float USER_SWAP_YZ_SIGN = +1.0f;

static Mat3 correctedUserUpperMatrix(const Vec3& rawX, const Vec3& rawY, const Vec3& rawZ) {
  Vec3 xAxis = normalize(rawX);

  if (!USER_SWAP_YZ_CORRECTION) {
    Vec3 yAxis = normalize(rawY);
    Vec3 zAxis = normalize(rawZ);
    return makeMatrixFromColumns(xAxis, yAxis, zAxis);
  }

  Vec3 yAxis = normalize(scale(rawZ, USER_SWAP_YZ_SIGN));
  Vec3 zAxis = normalize(scale(rawY, -USER_SWAP_YZ_SIGN));
  return makeMatrixFromColumns(xAxis, yAxis, zAxis);
}

static Mat3 worldFrameToStartupRobotRef(const Mat3& R_world) {
  return matMul(matTranspose(robotRefZero_world), R_world);
}

static const MotorMap& mapForId(uint8_t id) {
  if (id == DXL_ID_Q1) return MAP_Q1;
if (id == DXL_ID_Q2) return MAP_Q2;
if (id == DXL_ID_Q3) return MAP_Q3;
if (id == DXL_ID_Q4) return MAP_Q4;
if (id == DXL_ID_Q5) return MAP_Q5;
if (id == DXL_ID_Q6) return MAP_Q6;
return MAP_Q7;
}

static float deltaLimitDeg(uint8_t id, float deg) {
  if (id == DXL_ID_Q1) return clampf(deg, Q1_DELTA_MIN_DEG, Q1_DELTA_MAX_DEG);
  if (id == DXL_ID_Q2) return clampf(deg, Q2_DELTA_MIN_DEG, Q2_DELTA_MAX_DEG);
  if (id == DXL_ID_Q3) return clampf(deg, Q3_DELTA_MIN_DEG, Q3_DELTA_MAX_DEG);
  if (id == DXL_ID_Q4) return clampf(deg, Q4_DELTA_MIN_DEG, Q4_DELTA_MAX_DEG);
  if (id == DXL_ID_Q5) return clampf(deg, Q5_DELTA_MIN_DEG, Q5_DELTA_MAX_DEG);
  if (id == DXL_ID_Q6) return clampf(deg, Q6_DELTA_MIN_DEG, Q6_DELTA_MAX_DEG);
  return clampf(deg, Q7_DELTA_MIN_DEG, Q7_DELTA_MAX_DEG);
}

static void setMotorTargetFromDelta(uint8_t id, float qDeltaRad) {
  
  if (!DRIVE_MOTORS) return;
  float qDeltaDeg = deltaLimitDeg(id, qDeltaRad * RAD2DEG);
  MotorMap mp = mapForId(id);
  float targetRawDeg = motorStartRawDeg[id] + mp.sign * mp.gear * qDeltaDeg;
  if (id == DXL_ID_Q1 || id == DXL_ID_Q2 || id == DXL_ID_Q3) {
  dxl2.setGoalPosition(id, targetRawDeg, UNIT_DEGREE);
} else {
  dxl1.setGoalPosition(id, targetRawDeg, UNIT_DEGREE);
}
}

static bool looksLikeDatasetStart(char c) {
  return c == '0' || c == '1' || c == '2' || c == '3';
}

static bool parseDatasetLine(char* s) {
  while (*s == ' ' || *s == '\t') {
    s++;
  }

  uint8_t datasetId;

  if (s[0] == '0' && s[1] == '0' && s[2] == '0' && s[3] == '0') {
    datasetId = 0;
  } else if (s[0] == '1' && s[1] == '1' && s[2] == '1' && s[3] == '1') {
    datasetId = 1;
  } else if (s[0] == '2' && s[1] == '2' && s[2] == '2' && s[3] == '2') {
    datasetId = 2;
  } else if (s[0] == '3' && s[1] == '3' && s[2] == '3' && s[3] == '3') {
    datasetId = 3;
  } else {
    rxBadTag++;
    return false;
  }

  char* p = s + 4;
  float v[9];

  for (int i = 0; i < 9; i++) {
    while (*p == ' ' || *p == '\t') {
      p++;
    }

    char* endPtr;
    v[i] = strtof(p, &endPtr);

    if (endPtr == p) {
      rxBadFormat++;
      return false;
    }

    if (!isfinite(v[i])) {
      rxBadFormat++;
      return false;
    }

    p = endPtr;
  }

  Vec3 rawX = makeVec3(v[0], v[1], v[2]);
  Vec3 rawY = makeVec3(v[3], v[4], v[5]);
  Vec3 rawZ = makeVec3(v[6], v[7], v[8]);

  if (norm(rawX) < 0.5f || norm(rawY) < 0.5f || norm(rawZ) < 0.5f) {
    rxBadFormat++;
    return false;
  }

  Vec3 xAxis = normalize(rawX);
  Vec3 yAxis = normalize(rawY);
  Vec3 zAxis = normalize(rawZ);

  uint32_t now = millis();

  if (datasetId == 0) {
    robotRefFrame.R_world = makeMatrixFromColumns(xAxis, yAxis, zAxis);
    robotRefFrame.valid = true;
    robotRefFrame.lastMs = now;
  } else if (datasetId == 1) {
    userUpperFrame.R_world = correctedUserUpperMatrix(xAxis, yAxis, zAxis);
    userUpperFrame.valid = true;
    userUpperFrame.lastMs = now;
  } else if (datasetId == 2) {
    userForearmFrame.R_world = makeMatrixFromColumns(xAxis, yAxis, zAxis);
    userForearmFrame.valid = true;
    userForearmFrame.lastMs = now;
  } else if (datasetId == 3) {
    userPalmFrame.R_world = makeMatrixFromColumns(xAxis, yAxis, zAxis);
    userPalmFrame.valid = true;
    userPalmFrame.lastMs = now;
    rxPalm3333++;
  }

  rxGoodLines++;
  return true;
}

static void readReferenceSerial() {
  static char line[RX_LINE_MAX];
  static size_t idx = 0;
  static bool haveTagStart = false;

  const uint32_t startUs = micros();
  uint16_t bytesRead = 0;

  while (REF_SERIAL.available()) {
    if (bytesRead++ >= RX_MAX_BYTES_PER_LOOP) break;
    if ((uint32_t)(micros() - startUs) >= RX_PROCESS_BUDGET_US) break;

    char c = (char)REF_SERIAL.read();

    if (c == '\r') continue;

    // Before accepting a line, resync to a plausible dataset tag start.
    // This prevents debug text or a chopped packet from poisoning the parser.
    if (!haveTagStart) {
      if (!looksLikeDatasetStart(c)) {
        rxResyncDrops++;
        continue;
      }
      haveTagStart = true;
      idx = 0;
    }

    if (c == '\n') {
      line[idx] = '\0';
      if (idx >= 4) parseDatasetLine(line);
      idx = 0;
      haveTagStart = false;
      continue;
    }

    if (idx < RX_LINE_MAX - 1) {
      line[idx++] = c;
    } else {
      // Oversized line means the stream is corrupt or not newline-terminated.
      // Drop the partial line and resync at the next plausible packet start.
      rxOverflow++;
      idx = 0;
      haveTagStart = false;
    }
  }
}

static bool initMotor(uint8_t id) {
  bool ok = true;
  Dynamixel2Arduino* bus = nullptr;

  if (id == DXL_ID_Q1 || id == DXL_ID_Q2 || id == DXL_ID_Q3) {
    dxl2.setPortProtocolVersion(2.0f);
    bus = &dxl2;
  } else {
    dxl1.setPortProtocolVersion(1.0f);
    bus = &dxl1;
  }

  ok &= bus->ping(id);
  delay(20);

  ok &= bus->torqueOff(id);
  delay(20);

  ok &= bus->setOperatingMode(id, OP_EXTENDED_POSITION);
  delay(20);

  ok &= bus->torqueOn(id);
  delay(20);

  float pos = bus->getPresentPosition(id, UNIT_DEGREE);
  if (!isfinite(pos)) ok = false;

  motorStartRawDeg[id] = pos;

  DEBUG_SERIAL.print("Motor ID ");
  DEBUG_SERIAL.print(id);
  DEBUG_SERIAL.print(" Protocol ");
  DEBUG_SERIAL.print((bus == &dxl2) ? "2.0" : "1.0");
  DEBUG_SERIAL.print(ok ? " OK, start raw deg = " : " FAIL, raw deg = ");
  DEBUG_SERIAL.println(pos, 3);

  return ok;
}

static bool initXHMotors() {
  dxl2.setPortProtocolVersion(2.0f);
  bool ok = true;
  ok &= initMotor(DXL_ID_Q1);
  ok &= initMotor(DXL_ID_Q2);
  ok &= initMotor(DXL_ID_Q3);
  return ok;
}

static bool initMXMotors() {
  dxl1.setPortProtocolVersion(1.0f);
  bool ok = true;
  ok &= initMotor(DXL_ID_Q4);
  ok &= initMotor(DXL_ID_Q5);
  ok &= initMotor(DXL_ID_Q6);
  ok &= initMotor(DXL_ID_Q7);
  return ok;
}

static bool tryCalibrate() {
  static uint32_t freshSinceMs = 0;
  static bool freshWindowStarted = false;

  if (calibrated) return true;

  uint32_t now = millis();

  if (!imuDataFresh(now)) {
    freshSinceMs = 0;
    freshWindowStarted = false;
    return false;
  }

  if (!freshWindowStarted) {
    freshWindowStarted = true;
    freshSinceMs = now;
    return false;
  }

  if (now - freshSinceMs < 300) {
    return false;
  }

  robotRefZero_world = robotRefFrame.R_world;

  userUpperZero_ref = worldFrameToStartupRobotRef(userUpperFrame.R_world);
  userUpperZero_ref_T = matTranspose(userUpperZero_ref);

  userForearmZero_ref = worldFrameToStartupRobotRef(userForearmFrame.R_world);
  userForearmZero_ref_T = matTranspose(userForearmZero_ref);

  userPalmZero_ref = worldFrameToStartupRobotRef(userPalmFrame.R_world);
  userPalmZero_ref_T = matTranspose(userPalmZero_ref);

  resetMotionState();

  calibrated = true;

  DEBUG_SERIAL.println("\n=== MOTION CALIBRATION CAPTURED ===");
  DEBUG_SERIAL.println("Initial user connection accepted after stable fresh IMU window.");
  DEBUG_SERIAL.println("User pose captured as zero command.");
  DEBUG_SERIAL.println("Motion state reset before first motor command.");

  return true;
}

static void computeTarget() {
  // Convert current user upper-arm IMU frame into the frozen robot reference frame.
  Mat3 RuserNow_ref = worldFrameToStartupRobotRef(userUpperFrame.R_world);

  // Express current user body axes in the USER STARTUP frame.
  // At calibration: xRel=[1,0,0], yRel=[0,1,0], zRel=[0,0,1].
  Vec3 xNow_ref = matCol(RuserNow_ref, 0); // user +X = along upper arm
  Vec3 yNow_ref = matCol(RuserNow_ref, 1); // perpendicular axis used for roll/twist

  Vec3 xRel = normalize(matVec(userUpperZero_ref_T, xNow_ref));
  Vec3 yRel = normalize(matVec(userUpperZero_ref_T, yNow_ref));

  // Shoulder model for deltas from startup:
  //   Rdelta = Rz(q1) * Ry(q2) * Rx(q3)
  // The arm direction is transformed +X:
  //   xRel = [cos(q1)cos(q2), sin(q1)cos(q2), -sin(q2)]
  float q2 = asinf(clampf(-xRel.z, -1.0f, 1.0f));
  float q1 = atan2f(xRel.y, xRel.x);

  // Roll/twist: derive roll from vector rotation, not from Euler decomposition.
  //
  // First compute the shortest-arc "swing" that moves the startup arm axis (+X)
  // onto the current arm axis xRel. That swing has no twist around the arm.
  // Then compare where startup +Y would be after that swing against the actual
  // current +Y, projected onto the plane perpendicular to the arm.
  const Vec3 xZero = makeVec3(1.0f, 0.0f, 0.0f);
  const Vec3 yZero = makeVec3(0.0f, 1.0f, 0.0f);

  Vec3 ySwing = swingRotateVector(xZero, xRel, yZero);
  Vec3 yNoRollRaw = projectPerp(ySwing, xRel);
  Vec3 yActualRaw = projectPerp(yRel, xRel);

  float q3 = 0.0f;
  if (norm(yNoRollRaw) > 1e-5f && norm(yActualRaw) > 1e-5f) {
    Vec3 yNoRoll = normalize(yNoRollRaw);
    Vec3 yActual = normalize(yActualRaw);
    q3 = signedAngleAroundAxis(yNoRoll, yActual, xRel);
    q3 = unwrapNear(q3, qTarget.q3);
  }

  ShoulderDelta target{};
  target.q1 = wrapPi(SHOULDER_Q1_MATH_SIGN * q1);
  target.q2 = wrapPi(SHOULDER_Q2_MATH_SIGN * q2);
  target.q3 = wrapPi(SHOULDER_Q3_MATH_SIGN * q3);

  if (!isfinite(target.q1) || !isfinite(target.q2) || !isfinite(target.q3)) return;

  qTarget.q1 = clampf(target.q1, Q1_DELTA_MIN_DEG * DEG2RAD, Q1_DELTA_MAX_DEG * DEG2RAD);
  qTarget.q2 = clampf(target.q2, Q2_DELTA_MIN_DEG * DEG2RAD, Q2_DELTA_MAX_DEG * DEG2RAD);
  qTarget.q3 = clampf(target.q3, Q3_DELTA_MIN_DEG * DEG2RAD, Q3_DELTA_MAX_DEG * DEG2RAD);
}

static void computeElbowTarget() {
  Mat3 RupperNow_ref = worldFrameToStartupRobotRef(userUpperFrame.R_world);
  Mat3 RforeNow_ref  = worldFrameToStartupRobotRef(userForearmFrame.R_world);
  Mat3 RpalmNow_ref  = worldFrameToStartupRobotRef(userPalmFrame.R_world);

  Mat3 RupperDelta = matMul(userUpperZero_ref_T, RupperNow_ref);
  Mat3 RforeDelta  = matMul(userForearmZero_ref_T, RforeNow_ref);
  Mat3 RpalmDelta  = matMul(userPalmZero_ref_T, RpalmNow_ref);

  // Q4: elbow flexion from forearm relative to upper arm
  Mat3 RrelElbow = matMul(matTranspose(RupperDelta), RforeDelta);
  Vec3 xRel = normalize(matCol(RrelElbow, 0));

  float q4 = atan2f(xRel.y, xRel.x);

  // Q5: forearm rotation from palm/hand relative to forearm
  Mat3 RrelPalm = matMul(matTranspose(RforeDelta), RpalmDelta);

  float q5 = atan2f(RrelPalm.m[2][1], RrelPalm.m[1][1]);
  q5 = unwrapNear(q5, q5ForWristReference);

  // Raw geometric Q5, used only for wrist reference correction
  q5ForWristReference = clampf(wrapPi(q5),
                               Q5_DELTA_MIN_DEG * DEG2RAD,
                               Q5_DELTA_MAX_DEG * DEG2RAD);

  // Signed commanded Q5, used for motor target
  qElbowTarget.q5 = clampf(wrapPi(ELBOW_Q5_MATH_SIGN * q5),
                           Q5_DELTA_MIN_DEG * DEG2RAD,
                           Q5_DELTA_MAX_DEG * DEG2RAD);

  qElbowTarget.q4 = clampf(wrapPi(ELBOW_Q4_MATH_SIGN * q4),
                           Q4_DELTA_MIN_DEG * DEG2RAD,
                           Q4_DELTA_MAX_DEG * DEG2RAD);
}



static void computeWristTarget() {
  Mat3 RforeNow_ref = worldFrameToStartupRobotRef(userForearmFrame.R_world);
  Mat3 RpalmNow_ref = worldFrameToStartupRobotRef(userPalmFrame.R_world);

  Mat3 RforeDelta = matMul(userForearmZero_ref_T, RforeNow_ref);
  Mat3 RpalmDelta = matMul(userPalmZero_ref_T, RpalmNow_ref);

  // Palm motion relative to previous link / forearm
  Mat3 RforeRolled = matMul(RforeDelta, rotX(q5ForWristReference));
  Mat3 Rwrist = matMul(matTranspose(RforeRolled), RpalmDelta);

  // Motor 6: around Z axis, left/right
  float q6 = atan2f(Rwrist.m[1][0], Rwrist.m[0][0]);

  float q7 = atan2f(
    -Rwrist.m[2][0],
    sqrtf(Rwrist.m[2][1]*Rwrist.m[2][1] + Rwrist.m[2][2]*Rwrist.m[2][2])
  );

  q6 = unwrapNear(q6, qWristTarget.q6);
  q7 = unwrapNear(q7, qWristTarget.q7);


  qWristTarget.q6 = clampf(wrapPi(WRIST_Q6_MATH_SIGN * q6),
                           Q6_DELTA_MIN_DEG * DEG2RAD,
                           Q6_DELTA_MAX_DEG * DEG2RAD);

  qWristTarget.q7 = clampf(wrapPi(WRIST_Q7_MATH_SIGN * q7),
                           Q7_DELTA_MIN_DEG * DEG2RAD,
                           Q7_DELTA_MAX_DEG * DEG2RAD);
}

float q4_hist[N] = {0};
float q5_hist[N] = {0};
int idx = 0;

float avg_q4 = 0.0f;
float avg_q5 = 0.0f;

void updateElbowFilter(float new_q4, float new_q5) {
  avg_q4 -= q4_hist[idx] / N;
  avg_q5 -= q5_hist[idx] / N;

  q4_hist[idx] = new_q4;
  q5_hist[idx] = new_q5;

  avg_q4 += new_q4 / N;
  avg_q5 += new_q5 / N;

  idx = (idx + 1) % N;
}

static void slewCommandTowardTarget() {
  qCmd.q1 = stepToward(qCmd.q1, qTarget.q1, MAX_STEP_Q1_DEG * DEG2RAD);
  qCmd.q2 = stepToward(qCmd.q2, qTarget.q2, MAX_STEP_Q2_DEG * DEG2RAD);
  qCmd.q3 = stepToward(qCmd.q3, qTarget.q3, MAX_STEP_Q3_DEG * DEG2RAD);
}

static void slewElbowCommandTowardTarget() {
  qElbowCmd.q4 = stepToward(qElbowCmd.q4, qElbowTarget.q4, MAX_STEP_Q4_DEG * DEG2RAD);
  qElbowCmd.q5 = stepToward(qElbowCmd.q5, qElbowTarget.q5, MAX_STEP_Q5_DEG * DEG2RAD);
}

static void sendShoulderTargets() {
  setMotorTargetFromDelta(DXL_ID_Q1, qCmd.q1);
  setMotorTargetFromDelta(DXL_ID_Q2, qCmd.q2);
  setMotorTargetFromDelta(DXL_ID_Q3, qCmd.q3);
}

static void sendElbowTargets() {
  setMotorTargetFromDelta(DXL_ID_Q4, qElbowCmd.q4);
  setMotorTargetFromDelta(DXL_ID_Q5, qElbowCmd.q5);
}

static void sendWristTargets() {
  setMotorTargetFromDelta(DXL_ID_Q6, qWristCmd.q6);
  setMotorTargetFromDelta(DXL_ID_Q7, qWristCmd.q7);
}

static void debugPrint() {
  uint32_t now = millis();
  if (now - lastDebugMs < 200) return;
  lastDebugMs = now;

  DEBUG_SERIAL.print("cal="); DEBUG_SERIAL.print(calibrated ? "YES" : "NO");
  DEBUG_SERIAL.print(" ref="); DEBUG_SERIAL.print(robotRefFrame.valid ? "OK" : "NO");
  DEBUG_SERIAL.print(" upper="); DEBUG_SERIAL.print(userUpperFrame.valid ? "OK" : "NO");
  DEBUG_SERIAL.print(" forearm="); DEBUG_SERIAL.print(userForearmFrame.valid ? "OK" : "NO");

  DEBUG_SERIAL.print(" | target deg q1/q2/q3 = ");
  DEBUG_SERIAL.print(qTarget.q1 * RAD2DEG, 2); DEBUG_SERIAL.print(" / ");
  DEBUG_SERIAL.print(qTarget.q2 * RAD2DEG, 2); DEBUG_SERIAL.print(" / ");
  DEBUG_SERIAL.print(qTarget.q3 * RAD2DEG, 2);

  DEBUG_SERIAL.print(" | cmd deg q1/q2/q3 = ");
  DEBUG_SERIAL.print(qCmd.q1 * RAD2DEG, 2); DEBUG_SERIAL.print(" / ");
  DEBUG_SERIAL.print(qCmd.q2 * RAD2DEG, 2); DEBUG_SERIAL.print(" / ");
  DEBUG_SERIAL.print(qCmd.q3 * RAD2DEG, 2);

  DEBUG_SERIAL.print(" | q4/q5 target deg = ");
  DEBUG_SERIAL.print(qElbowTarget.q4 * RAD2DEG, 2);
  DEBUG_SERIAL.print(" / ");
  DEBUG_SERIAL.print(qElbowTarget.q5 * RAD2DEG, 2);

  DEBUG_SERIAL.print(" | q4/q5 cmd deg = ");
  DEBUG_SERIAL.print(qElbowCmd.q4 * RAD2DEG, 2);
  DEBUG_SERIAL.print(" / ");
  DEBUG_SERIAL.print(qElbowCmd.q5 * RAD2DEG, 2);

  DEBUG_SERIAL.print(" | q6/q7 wrist target deg = ");
  DEBUG_SERIAL.print(qWristTarget.q6 * RAD2DEG, 2);
  DEBUG_SERIAL.print(" / ");
  DEBUG_SERIAL.print(qWristTarget.q7 * RAD2DEG, 2);

  DEBUG_SERIAL.print(" | motor raw target deg = ");
  DEBUG_SERIAL.print(motorStartRawDeg[1] + MAP_Q1.sign * MAP_Q1.gear * qCmd.q1 * RAD2DEG, 2); DEBUG_SERIAL.print(" / ");
  DEBUG_SERIAL.print(motorStartRawDeg[2] + MAP_Q2.sign * MAP_Q2.gear * qCmd.q2 * RAD2DEG, 2); DEBUG_SERIAL.print(" / ");
  DEBUG_SERIAL.print(motorStartRawDeg[3] + MAP_Q3.sign * MAP_Q3.gear * qCmd.q3 * RAD2DEG, 2);

  DEBUG_SERIAL.print(" | age ms ref/upper/forearm/palm = ");
  DEBUG_SERIAL.print(robotRefFrame.valid ? (now - robotRefFrame.lastMs) : 999999UL);
  DEBUG_SERIAL.print(" / ");
  DEBUG_SERIAL.print(userUpperFrame.valid ? (now - userUpperFrame.lastMs) : 999999UL);
  DEBUG_SERIAL.print(" / ");
  DEBUG_SERIAL.print(userForearmFrame.valid ? (now - userForearmFrame.lastMs) : 999999UL);
  DEBUG_SERIAL.print(" / ");
  DEBUG_SERIAL.print(userPalmFrame.valid ? (now - userPalmFrame.lastMs) : 999999UL);

  DEBUG_SERIAL.print(" | rx good/3333/badTag/badFmt/ovf/resync = ");
  DEBUG_SERIAL.print(rxGoodLines); DEBUG_SERIAL.print("/");
  DEBUG_SERIAL.print(rxPalm3333); DEBUG_SERIAL.print("/");
  DEBUG_SERIAL.print(rxBadTag); DEBUG_SERIAL.print("/");
  DEBUG_SERIAL.print(rxBadFormat); DEBUG_SERIAL.print("/");
  DEBUG_SERIAL.print(rxOverflow); DEBUG_SERIAL.print("/");
  DEBUG_SERIAL.println(rxResyncDrops);
}

static bool imuDataFresh(uint32_t now) {
  static constexpr uint32_t MAX_IMU_AGE_MS = 2000;

  return
    robotRefFrame.valid &&
    userUpperFrame.valid &&
    userForearmFrame.valid &&
    userPalmFrame.valid &&
    (now - robotRefFrame.lastMs <= MAX_IMU_AGE_MS) &&
    (now - userUpperFrame.lastMs <= MAX_IMU_AGE_MS) &&
    (now - userForearmFrame.lastMs <= MAX_IMU_AGE_MS) &&
    (now - userPalmFrame.lastMs <= MAX_IMU_AGE_MS);
}

void requestAndReadEspBatch() {
  uint32_t before = rxGoodLines;

  digitalWrite(ESP_READY_PIN, HIGH);

  uint32_t startMs = millis();

  while (rxGoodLines < before + 4 && millis() - startMs < 25) {
    readReferenceSerial();
  }

  digitalWrite(ESP_READY_PIN, LOW);
}

static void resetMotionState() {
  qCmd = {};
  qTarget = {};

  qElbow = {};
  qElbowCmd = {};
  qElbowTarget = {};

  qWristCmd = {};
  qWristTarget = {};
  q5ForWristReference = 0.0f;

  shoulderQ1Filt = 0.0f;
  shoulderQ2Filt = 0.0f;
  shoulderQ3Filt = 0.0f;

  wristQ6Filt = 0.0f;
  wristQ7Filt = 0.0f;

  shoulderFilterInit = false;
  wristFilterInit = false;
  elbowFilterInit = false;

  for (int i = 0; i < N; i++) {
    q4_hist[i] = 0.0f;
    q5_hist[i] = 0.0f;
  }

  idx = 0;
  avg_q4 = 0.0f;
  avg_q5 = 0.0f;

  lastControlMs = millis();
}

void setup() {
  pinMode(ESP_READY_PIN, OUTPUT);
  digitalWrite(ESP_READY_PIN, LOW);
  DEBUG_SERIAL.begin(DBG_BAUD);
  REF_SERIAL.addMemoryForRead(refSerialRxExtraBuffer, sizeof(refSerialRxExtraBuffer));
  REF_SERIAL.begin(REF_BAUD);
  DXL_SERIAL.begin(DXL_BAUD);
  dxl2.begin(DXL_BAUD);
  dxl2.setPortProtocolVersion(2.0f);
  dxl1.begin(DXL_BAUD);
  dxl1.setPortProtocolVersion(1.0f);

  delay(1000);
  DEBUG_SERIAL.println("\n=== CONTROLLER START ===");
  DEBUG_SERIAL.println("Waiting for 0000, 1111, 2222, 3333 packets.");
  DEBUG_SERIAL.println("Dynamixel IDs 1-7 are controlled.");
  DEBUG_SERIAL.println("REF UART baud = 921600.");
  DEBUG_SERIAL.println(USER_SWAP_YZ_CORRECTION ? "User IMU correction: X=X, Y=Z, Z=-Y" : "User IMU correction: OFF");
  DEBUG_SERIAL.println("Shoulder model: R_delta = Rz(q1) * Ry(q2) * Rx(q3)  [q1=ID1/+Z, q2=ID2/local +Y]");

  bool motorInitOk = true;
motorInitOk &= initXHMotors();
motorInitOk &= initMXMotors();

if (!motorInitOk) {
  DEBUG_SERIAL.println("MOTOR INIT FAILED. Controller will still parse/debug but motor commands may fail.");
}
}

void loop() {
  requestAndReadEspBatch();

  uint32_t now = millis();
  bool fresh = imuDataFresh(now);

  if (!calibrated) {
    if (!fresh) {
      resetMotionState();
      debugPrint();
      return;
    }

    tryCalibrate();

    if (!calibrated) {
      resetMotionState();
      debugPrint();
      return;
    }
  }

  if (!fresh) {
    debugPrint();
    return;
  }

  now = millis();

  if (now - lastControlMs >= 10) {
    lastControlMs = now;

    computeTarget();

    if (!shoulderFilterInit) {
      shoulderQ1Filt = qTarget.q1;
      shoulderQ2Filt = qTarget.q2;
      shoulderQ3Filt = qTarget.q3;
      shoulderFilterInit = true;
    } else {
      shoulderQ1Filt += SHOULDER_LPF_ALPHA * (qTarget.q1 - shoulderQ1Filt);
      shoulderQ2Filt += SHOULDER_LPF_ALPHA * (qTarget.q2 - shoulderQ2Filt);
      shoulderQ3Filt += SHOULDER_LPF_ALPHA * (qTarget.q3 - shoulderQ3Filt);
    }

    qTarget.q1 = shoulderQ1Filt;
    qTarget.q2 = shoulderQ2Filt;
    qTarget.q3 = shoulderQ3Filt;

    slewCommandTowardTarget();
    sendShoulderTargets();

    computeElbowTarget();
    computeWristTarget();

    slewElbowCommandTowardTarget();
    sendElbowTargets();

    if (!wristFilterInit) {
      wristQ6Filt = qWristTarget.q6;
      wristQ7Filt = qWristTarget.q7;
      wristFilterInit = true;
    } else {
      wristQ6Filt += WRIST_LPF_ALPHA * (qWristTarget.q6 - wristQ6Filt);
      wristQ7Filt += WRIST_LPF_ALPHA * (qWristTarget.q7 - wristQ7Filt);
    }

    qWristCmd.q6 = wristQ6Filt;
    qWristCmd.q7 = wristQ7Filt;

    sendWristTargets();
  }

  debugPrint();
}
