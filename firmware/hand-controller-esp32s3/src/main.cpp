//palm hand controller main code 1.1
#include <Arduino.h>
//#include <Adafruit_NeoPixel.h>

// ============================================================
// ESP32-S3 HAND CONTROLLER
// 3x TB6612FNG drivers, 6 DC motors
// ============================================================

// ----------------------------
// Fill in real ESP32-S3 GPIOs
// ----------------------------

// Driver 1
#define D1_PWMA  4
#define D1_AIN1  5
#define D1_AIN2  6
#define D1_BIN1  7
#define D1_BIN2  8
#define D1_PWMB  9

// Driver 2
#define D2_PWMA  15
#define D2_AIN2  10
#define D2_AIN1  11
#define D2_BIN1  12
#define D2_BIN2  13
#define D2_PWMB  14

// Driver 3
#define D3_PWMA  16
#define D3_AIN2  17
#define D3_AIN1  18
#define D3_BIN1  2
#define D3_BIN2  42
#define D3_PWMB  19
#define SWITCH_DEBUG_LED 46

bool homeSwitchState[6] = {false, false, false, false, false, false};


uint8_t muxScanChannel = 0;
uint32_t lastMuxScanUs = 0;
static constexpr uint32_t MUX_SCAN_INTERVAL_US = 300;


long motorMinCount[6] = {0, 0, 0, 0, 0, 0};

long motorMaxCount[6] = {
  12285,  // motor 1 thumb flex, screw drive, 13 mm
  233,    // motor 2 thumb rotation, 70 deg
  12285,  // motor 3 index
  12285,  // motor 4 middle
  12285,  // motor 5 ring
  12285   // motor 6 pinkie
};

long motorSoftMaxCount[6] = {
  11000,
  220,
  11000,
  11000,
  11000,
  11000
};

// ============================================================
// calibration functions
// ============================================================

static constexpr float HOME_RATIO[6] = {
  0.00f,  // motor 1 thumb flex
  0.00f,  // motor 2 thumb rotation
  0.85f,  // motor 3 index
  0.85f,  // motor 4 middle
  0.85f,  // motor 5 ring
  0.85f   // motor 6 pinkie
};

static constexpr float CLEAR_RATIO[6] = {
  0.05f,
  0.05f,
  0.80f,
  0.80f,
  0.80f,
  0.80f
};

static constexpr float MIDDLE_RATIO[6] = {
  0.50f,
  0.50f,
  0.50f,
  0.50f,
  0.50f,
  0.50f
};

void setEncoderCount(uint8_t motorIndex, long value);
long ratioToCounts(uint8_t motorIndex, float ratio);
void syncPidToCurrentPosition(uint8_t motorIndex);

bool moveUntilSwitchReleased(uint8_t motorIndex, uint32_t timeoutMs);
bool moveUntilSwitchPressed(uint8_t motorIndex, uint32_t timeoutMs);

bool calibrateAxisBySwitchEdge(uint8_t motorIndex);
bool moveAxisToRatio(uint8_t motorIndex, float ratio, uint32_t timeoutMs);
bool startupCalibrationSequence();


// ============================================================
// ENCODERS
// ============================================================


// Motor 2 (full quadrature)
#define ENC2_A 35
#define ENC2_B 36

// Motor 1 
#define ENC1_A 37

// Motor 3
#define ENC3_A 38

// Motor 4
#define ENC4_A 39

// Motor 5
#define ENC5_A 40

// Motor 6
#define ENC6_A 41

volatile long encoderCount[6] = {0, 0, 0, 0, 0, 0};
// For single-channel encoders.
// Updated by setMotor() later.
volatile int motorDirection[6] = {0, 0, 0, 0, 0, 0};

bool handReady = false;

#pragma pack(push, 1)

struct HandCommandPacket {
  uint8_t start;        // 0xFA
  int32_t target[6];    // target encoder counts
  uint8_t enableMask;   // bit 0 = motor1, bit 1 = motor2, etc.
  uint8_t crc;
};

struct HandTelemetryPacket {
  uint8_t start;         // 0xFB
  int32_t position[6];   // encoder counts
  int16_t pwm[6];        // current motor outputs
  uint8_t status;        // basic state flags
  uint8_t crc;
};

#pragma pack(pop)

// ============================================================
// PID MOTOR CONTROL LAYER
// For 6 DC motors with encoder position feedback
// ============================================================

struct MotorPID {
  long target = 0;          // desired encoder count
  long position = 0;        // current encoder count

  float kp = 1.0f;
  float ki = 0.0f;
  float kd = 0.0f;

  float integral = 0.0f;
  float lastError = 0.0f;

  int output = 0;           // -255 to +255

  int maxOutput = 255;
  int minOutput = -255;

  float integralLimit = 500.0f;

  bool enabled = false;
};

MotorPID motorPID[6];

// Motor 1  single-channel
void IRAM_ATTR enc1ISR() {
  encoderCount[0] += motorDirection[0];
}

// Motor 2 quadrature
void IRAM_ATTR enc2ISR() {
  bool a = digitalRead(ENC2_A);
  bool b = digitalRead(ENC2_B);

  if (a == b) {
    encoderCount[1]++;
  } else {
    encoderCount[1]--;
  }
}

// Motor 3 single-channel
void IRAM_ATTR enc3ISR() {
  encoderCount[2] += motorDirection[2];
}

void IRAM_ATTR enc4ISR() {
  encoderCount[3] += motorDirection[3];
}

void IRAM_ATTR enc5ISR() {
  encoderCount[4] += motorDirection[4];
}

void IRAM_ATTR enc6ISR() {
  encoderCount[5] += motorDirection[5];
}

void initEncoders() {
  pinMode(ENC1_A, INPUT);

  pinMode(ENC2_A, INPUT);
  pinMode(ENC2_B, INPUT);

  pinMode(ENC3_A, INPUT);
  pinMode(ENC4_A, INPUT);
  pinMode(ENC5_A, INPUT);
  pinMode(ENC6_A, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENC1_A), enc1ISR, RISING);

  // Quadrature: interrupt on A, read B for direction
  attachInterrupt(digitalPinToInterrupt(ENC2_A), enc2ISR, CHANGE);

  attachInterrupt(digitalPinToInterrupt(ENC3_A), enc3ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC4_A), enc4ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC5_A), enc5ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC6_A), enc6ISR, RISING);
}

long getEncoderCount(uint8_t motorIndex) {
  if (motorIndex >= 6) return 0;

  noInterrupts();
  long value = encoderCount[motorIndex];
  interrupts();

  return value;
}

void resetEncoder(uint8_t motorIndex) {
  if (motorIndex >= 6) return;

  noInterrupts();
  encoderCount[motorIndex] = 0;
  interrupts();
}

void resetAllEncoders() {
  noInterrupts();
  for (int i = 0; i < 6; i++) {
    encoderCount[i] = 0;
  }
  interrupts();
}


// ============================================================
// COMMUNICATION LAYER
// ============================================================

#define HAND_SERIAL Serial1
static constexpr uint32_t HAND_BAUD = 921600;

// Set these later after UART GPIO is finalized
#define HAND_RX_PIN  RX
#define HAND_TX_PIN  TX

static constexpr uint8_t CMD_START = 0xFA;
static constexpr uint8_t TEL_START = 0xFB;

uint8_t crcXor(const uint8_t* data, size_t len) {
  uint8_t c = 0;
  for (size_t i = 0; i < len; i++) {
    c ^= data[i];
  }
  return c;
}

void initCommunication() {
  HAND_SERIAL.begin(HAND_BAUD, SERIAL_8N1, HAND_RX_PIN, HAND_TX_PIN);
}

bool readHandCommand(HandCommandPacket &cmd) {
  while (HAND_SERIAL.available() > 0) {
    if (HAND_SERIAL.peek() != CMD_START) {
      HAND_SERIAL.read();
      continue;
    }

    if (HAND_SERIAL.available() < sizeof(HandCommandPacket)) {
      return false;
    }

    HAND_SERIAL.readBytes((uint8_t*)&cmd, sizeof(HandCommandPacket));

    uint8_t calc = crcXor((uint8_t*)&cmd, sizeof(HandCommandPacket) - 1);

    if (calc != cmd.crc) {
      return false;
    }

    return true;
  }

  return false;
}



static uint32_t lastTelemetryMs = 0;
static constexpr uint32_t TELEMETRY_INTERVAL_MS = 20; // 50 Hz



long position;   // encoder count
long target;     // desired encoder count
int output;      // motor PWM command, -255 to +255

// ============================================================
// Motor abstraction
// ============================================================

struct MotorPins {
  uint8_t pwm;
  uint8_t in1;
  uint8_t in2;
};

// Motor order: finger/thumb motor 1–6
MotorPins motors[6] = {
  {D1_PWMA, D1_AIN1, D1_AIN2},  // Motor 1
  {D1_PWMB, D1_BIN1, D1_BIN2},  // Motor 2

  {D2_PWMA, D2_AIN1, D2_AIN2},  // Motor 3
  {D2_PWMB, D2_BIN1, D2_BIN2},  // Motor 4

  {D3_PWMA, D3_AIN1, D3_AIN2},  // Motor 5
  {D3_PWMB, D3_BIN1, D3_BIN2}   // Motor 6
};

static constexpr int8_t MOTOR_POLARITY[6] = {
  +1,  // motor 1
  +1,  // motor 2 reversed
  +1,  // motor 3
  -1,  // motor 4
  +1,  // motor 5
  +1   // motor 6
};

// PWM range: -255 to +255
static constexpr int PWM_MAX = 255;

// ============================================================
// Motor functions
// ============================================================

void initMotors() {

analogWriteResolution(8);  // 0–255 PWM

  for (int i = 0; i < 6; i++) {
    pinMode(motors[i].pwm, OUTPUT);
    pinMode(motors[i].in1, OUTPUT);
    pinMode(motors[i].in2, OUTPUT);
    
    digitalWrite(motors[i].in1, LOW);
    digitalWrite(motors[i].in2, LOW);
    analogWrite(motors[i].pwm, 0);
  }
}

// speed:
//   +255 = forward
//    0   = stop / coast
//   -255 = reverse
void setMotor(uint8_t motorIndex, int speed) {
  if (motorIndex >= 6) return;

  speed = constrain(speed, -PWM_MAX, PWM_MAX);
  speed *= MOTOR_POLARITY[motorIndex];

  uint8_t pwm = motors[motorIndex].pwm;
  uint8_t in1 = motors[motorIndex].in1;
  uint8_t in2 = motors[motorIndex].in2;

  if (speed > 0) {
    motorDirection[motorIndex] = +1;

    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwm, speed);
  }
  else if (speed < 0) {
    motorDirection[motorIndex] = -1;

    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(pwm, -speed);
  }
  else {
    motorDirection[motorIndex] = 0;

    analogWrite(pwm, 0);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

void printEncoderCounts() {
  static uint32_t lastPrint = 0;
  uint32_t now = millis();

  if (now - lastPrint < 200) return;
  lastPrint = now;

  Serial.print("ENC: ");
  for (int i = 0; i < 6; i++) {
    Serial.print(i + 1);
    Serial.print("=");
    Serial.print(getEncoderCount(i));
    Serial.print(" ");
  }
  Serial.println();
}


void stopAllMotors() {
  for (int i = 0; i < 6; i++) {
    setMotor(i, 0);
  }
}

int updatePID(MotorPID &m, float dt) {
  if (!m.enabled) {
    m.output = 0;
    m.integral = 0.0f;
    m.lastError = 0.0f;
    return 0;
  }

  float error = (float)(m.target - m.position);

  m.integral += error * dt;
  m.integral = constrain(m.integral, -m.integralLimit, m.integralLimit);

  float derivative = (error - m.lastError) / dt;
  m.lastError = error;

  float out =
    m.kp * error +
    m.ki * m.integral +
    m.kd * derivative;

  out = constrain(out, m.minOutput, m.maxOutput);

  m.output = (int)out;
  return m.output;
}

void updateAllPID(float dt) {
  for (int i = 0; i < 6; i++) {
    motorPID[i].position = getEncoderCount(i);

    int pwm = updatePID(motorPID[i], dt);

    setMotor(i, pwm);
  }
}

void setMotorTarget(uint8_t motorIndex, long targetCount) {
  if (motorIndex >= 6) return;

  targetCount = constrain(targetCount, motorMinCount[motorIndex], motorMaxCount[motorIndex]);

  motorPID[motorIndex].target = targetCount;
  motorPID[motorIndex].enabled = true;
}

void disableMotorPID(uint8_t motorIndex) {
  if (motorIndex >= 6) return;

  motorPID[motorIndex].enabled = false;
  motorPID[motorIndex].output = 0;
  motorPID[motorIndex].integral = 0.0f;
  motorPID[motorIndex].lastError = 0.0f;

  setMotor(motorIndex, 0);
}

void disableAllPID() {
  for (int i = 0; i < 6; i++) {
    disableMotorPID(i);
  }
}

static uint32_t lastPidMs = 0;
static constexpr uint32_t PID_INTERVAL_MS = 10;  // 100 Hz

void updatePidLoop() {
  uint32_t now = millis();

  if (now - lastPidMs >= PID_INTERVAL_MS) {
    float dt = (now - lastPidMs) / 1000.0f;
    lastPidMs = now;

    if (dt <= 0.0f) return;

    updateAllPID(dt);
  }
}

// ============================================================
// HOMING LAYER
// MUX channels 0–5 = homing switches for motors 1–6
// ============================================================

#define MUX_SIG 20

#define MUX_S0  21
#define MUX_S1  47
#define MUX_S2  48
#define MUX_S3  45

const int HOMING_PWM[6] = {
  -200,  // motor 1
   200,  // motor 2 
   200,  // motor 3
  -200,  // motor 4
   200,  // motor 5
  -200   // motor 6
};



static constexpr uint32_t HOMING_TIMEOUT_MS = 5000;

bool motorHomed[6] = {false, false, false, false, false, false};

void initMux() {
  pinMode(SWITCH_DEBUG_LED, OUTPUT);
  digitalWrite(SWITCH_DEBUG_LED, LOW);
  pinMode(MUX_SIG, INPUT_PULLDOWN);

  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(MUX_S3, OUTPUT);
}

void selectMuxChannel(uint8_t channel) {
  digitalWrite(MUX_S0, (channel >> 0) & 1);
  digitalWrite(MUX_S1, (channel >> 1) & 1);
  digitalWrite(MUX_S2, (channel >> 2) & 1);
  digitalWrite(MUX_S3, (channel >> 3) & 1);

  delayMicroseconds(50);
}

void updateHomeSwitches() {
  uint32_t nowUs = micros();

  if (nowUs - lastMuxScanUs < MUX_SCAN_INTERVAL_US) return;
  lastMuxScanUs = nowUs;

  selectMuxChannel(muxScanChannel);
  delayMicroseconds(50);

  homeSwitchState[muxScanChannel] = (digitalRead(MUX_SIG) == HIGH);

  muxScanChannel++;
  if (muxScanChannel >= 6) {
    muxScanChannel = 0;
  }
}

void updateSwitchDebugLed() {
  bool anySwitchHigh = false;

  for (uint8_t ch = 0; ch < 6; ch++) {
    selectMuxChannel(ch);
    delayMicroseconds(50);

    if (digitalRead(MUX_SIG) == HIGH) {
      anySwitchHigh = true;
      break;
    }
  }

  digitalWrite(SWITCH_DEBUG_LED, anySwitchHigh ? HIGH : LOW);
}

bool readHomeSwitch(uint8_t motorIndex) {
  if (motorIndex >= 6) return false;

  selectMuxChannel(motorIndex);
  delayMicroseconds(100);

  bool pressed = (digitalRead(MUX_SIG) == HIGH);

  digitalWrite(SWITCH_DEBUG_LED, pressed ? HIGH : LOW);

  return pressed;
}

void setEncoderCount(uint8_t motorIndex, long value) {
  if (motorIndex >= 6) return;

  noInterrupts();
  encoderCount[motorIndex] = value;
  interrupts();
}

long ratioToCounts(uint8_t motorIndex, float ratio) {
  if (motorIndex >= 6) return 0;

  ratio = constrain(ratio, 0.0f, 1.0f);
  return (long)(motorMaxCount[motorIndex] * ratio);
}

void syncPidToCurrentPosition(uint8_t motorIndex) {
  if (motorIndex >= 6) return;

  long pos = getEncoderCount(motorIndex);

  motorPID[motorIndex].position = pos;
  motorPID[motorIndex].target = pos;
  motorPID[motorIndex].integral = 0.0f;
  motorPID[motorIndex].lastError = 0.0f;
  motorPID[motorIndex].output = 0;
}

bool moveUntilSwitchReleased(uint8_t motorIndex, uint32_t timeoutMs) {
  if (motorIndex >= 6) return false;

  uint32_t startMs = millis();

  disableMotorPID(motorIndex);
  setMotor(motorIndex, -HOMING_PWM[motorIndex]);

  while (readHomeSwitch(motorIndex)) {
    if (millis() - startMs > timeoutMs) {
      setMotor(motorIndex, 0);
      return false;
    }

    delay(1);
  }

  setMotor(motorIndex, 0);
  delay(100);
  return true;
}

bool moveUntilSwitchPressed(uint8_t motorIndex, uint32_t timeoutMs) {
  if (motorIndex >= 6) return false;

  uint32_t startMs = millis();

  disableMotorPID(motorIndex);
  setMotor(motorIndex, HOMING_PWM[motorIndex]);

  while (!readHomeSwitch(motorIndex)) {
    if (millis() - startMs > timeoutMs) {
      setMotor(motorIndex, 0);
      return false;
    }

    delay(1);
  }

  setMotor(motorIndex, 0);
  delay(100);
  return true;
}

bool calibrateAxisBySwitchEdge(uint8_t motorIndex) {
  if (motorIndex >= 6) return false;

  Serial.print("Calibrating axis ");
  Serial.println(motorIndex + 1);

  disableMotorPID(motorIndex);
  setMotor(motorIndex, 0);
  delay(100);

  if (readHomeSwitch(motorIndex)) {
    Serial.println("Switch already pressed. Backing away...");

    if (!moveUntilSwitchReleased(motorIndex, 2500)) {
      Serial.println("Failed to release switch.");
      setMotor(motorIndex, 0);
      motorHomed[motorIndex] = false;
      return false;
    }
  }

  Serial.println("Approaching switch...");

  if (!moveUntilSwitchPressed(motorIndex, HOMING_TIMEOUT_MS)) {
    Serial.println("Failed to reach switch.");
    setMotor(motorIndex, 0);
    motorHomed[motorIndex] = false;
    return false;
  }

  long homeCounts = ratioToCounts(motorIndex, HOME_RATIO[motorIndex]);

  setEncoderCount(motorIndex, homeCounts);
  syncPidToCurrentPosition(motorIndex);

  motorHomed[motorIndex] = true;

  Serial.print("Axis ");
  Serial.print(motorIndex + 1);
  Serial.print(" calibrated at counts = ");
  Serial.println(homeCounts);

  return true;
}

bool moveAxisToRatio(uint8_t motorIndex, float ratio, uint32_t timeoutMs) {
  if (motorIndex >= 6) return false;

  long target = ratioToCounts(motorIndex, ratio);

  syncPidToCurrentPosition(motorIndex);
  setMotorTarget(motorIndex, target);

  uint32_t startMs = millis();

  while (millis() - startMs <= timeoutMs) {
    motorPID[motorIndex].position = getEncoderCount(motorIndex);

    long error = motorPID[motorIndex].target - motorPID[motorIndex].position;

    if (abs(error) < 30) {
      disableMotorPID(motorIndex);
      setMotor(motorIndex, 0);
      return true;
    }

    updatePidLoop();
    delay(1);
  }

  disableMotorPID(motorIndex);
  setMotor(motorIndex, 0);
  return false;
}

bool startupCalibrationSequence() {
  bool ok = true;

  stopAllMotors();
  disableAllPID();

  for (int i = 0; i < 6; i++) {
    motorHomed[i] = false;
  }

  Serial.println("Starting edge-based calibration...");

  for (int i = 0; i < 6; i++) {
    bool axisOk = calibrateAxisBySwitchEdge(i);

    if (!axisOk) {
      Serial.print("Axis ");
      Serial.print(i + 1);
      Serial.println(" calibration FAILED.");
      ok = false;
      break;
    }

    delay(200);
  }

  if (!ok) {
    stopAllMotors();
    disableAllPID();
    return false;
  }

  Serial.println("Moving axes to clearance positions...");

  for (int i = 0; i < 6; i++) {
    bool moved = moveAxisToRatio(i, CLEAR_RATIO[i], 3000);

    Serial.print("Axis ");
    Serial.print(i + 1);
    Serial.println(moved ? " cleared." : " clearance move FAILED.");

    delay(150);
  }

  Serial.println("Moving axes to middle positions...");

  for (int i = 0; i < 6; i++) {
    bool moved = moveAxisToRatio(i, MIDDLE_RATIO[i], 5000);

    Serial.print("Axis ");
    Serial.print(i + 1);
    Serial.println(moved ? " centered." : " middle move FAILED.");

    delay(150);
  }

  stopAllMotors();
  disableAllPID();

  Serial.println("Startup calibration complete.");
  return true;
}

long angleToCounts_Motor2(float angleDeg,
                          float encoderCountsPerMotorRev,
                          float gearRatio) {
  return (long)(angleDeg * encoderCountsPerMotorRev * gearRatio / 360.0f);
}

long travelToCounts_Worm(float travelMm,
                         float encoderCountsPerMotorRev,
                         float pitchMmPerRev) {
  return (long)(travelMm * encoderCountsPerMotorRev / pitchMmPerRev);
}


void applyHandCommand(const HandCommandPacket &cmd) {
  if (!handReady) {
    disableAllPID();
    stopAllMotors();
    return;
  }

  for (int i = 0; i < 6; i++) {
    bool enabled = cmd.enableMask & (1 << i);

    if (enabled) {
      setMotorTarget(i, cmd.target[i]);
      motorPID[i].enabled = true;
    } else {
      disableMotorPID(i);
    }
  }
}

void sendTelemetry() {
  HandTelemetryPacket tel;

  tel.start = TEL_START;

  for (int i = 0; i < 6; i++) {
    tel.position[i] = getEncoderCount(i);
    tel.pwm[i] = motorPID[i].output;
  }

  tel.status = 0;

  tel.crc = 0;
  tel.crc = crcXor((uint8_t*)&tel, sizeof(HandTelemetryPacket) - 1);

  HAND_SERIAL.write((uint8_t*)&tel, sizeof(HandTelemetryPacket));
}

void updateCommunication() {
  HandCommandPacket cmd;

  if (readHandCommand(cmd)) {
    applyHandCommand(cmd);
  }

  uint32_t now = millis();
  if (now - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryMs = now;
    sendTelemetry();
  }
}

/* // ============================================================
// RGB DEBUG LED — active LOW
// ============================================================

#define RGB_LED_PIN 48
static uint32_t ledBlinkTimer = 0;
static bool ledBlinkState = false;

Adafruit_NeoPixel rgb(1, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);

enum DebugLedState {
  LED_BOOT,
  LED_HOMING,
  LED_READY,
  LED_UART_LOST,
  LED_ERROR
};

DebugLedState ledState = LED_BOOT; */



/* void initDebugLed() {
  rgb.begin();
  rgb.clear();
  rgb.show();
} */

/* void setRgb(uint8_t r, uint8_t g, uint8_t b) {
  rgb.setPixelColor(0, rgb.Color(r, g, b));
  rgb.show();
}

void updateDebugLed() {
  uint32_t now = millis();

  switch (ledState) {
    case LED_BOOT:
      if (now - ledBlinkTimer >= 150) {
        ledBlinkTimer = now;
        ledBlinkState = !ledBlinkState;
        setRgb(ledBlinkState ? 40 : 0,
               ledBlinkState ? 40 : 0,
               ledBlinkState ? 40 : 0);
      }
      break;

    case LED_HOMING:
      setRgb(80, 80, 0);
      break;

    case LED_READY:
      setRgb(0, 80, 0);
      break;

    case LED_UART_LOST:
      if (now - ledBlinkTimer >= 250) {
        ledBlinkTimer = now;
        ledBlinkState = !ledBlinkState;
        setRgb(0, 0, ledBlinkState ? 80 : 0);
      }
      break;

    case LED_ERROR:
      if (now - ledBlinkTimer >= 250) {
        ledBlinkTimer = now;
        ledBlinkState = !ledBlinkState;
        setRgb(ledBlinkState ? 80 : 0, 0, 0);
      }
      break;
  }
} */


void setup() {
  Serial.begin(115200);
  delay(1000);

  //initDebugLed();
  //ledState = LED_BOOT;

  initMotors();
  stopAllMotors();

  initMux();
  initEncoders();
  initCommunication();

  resetAllEncoders();

  Serial.println("Starting sequential homing...");
  //ledState = LED_HOMING;
  for (int i = 0; i < 6; i++) {
  Serial.print("Home switch ");
  Serial.print(i + 1);
  Serial.print(" = ");
  Serial.println(readHomeSwitch(i));
}
  handReady = startupCalibrationSequence();

  if (handReady) {
    Serial.println("Homing successful.");
    //ledState = LED_READY;
  } else {
    Serial.println("Homing failed. Hand commands disabled.");
    stopAllMotors();
    disableAllPID();
    //ledState = LED_ERROR;
  }

  Serial.println("Hand controller started.");
}

void loop() {
  updatePidLoop();
  updateCommunication();
  printEncoderCounts();
  //updateDebugLed();
  updateHomeSwitches();
  updateSwitchDebugLed();
}