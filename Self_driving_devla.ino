/*
 * ============================================
 * DEVLA — WRO FUTURE ENGINEERS 2026
 * STM32F103C8T6 BLUE PILL
 * ROUND 1 — OPEN CHALLENGE
 * 
 * ARCHITECTURE:
 * STM32  = motor + servo + TOF sensors + color
 * RPi    = camera + lap counting + direction
 * UART   = communication between them
 * ============================================
 */

#include <Servo.h>
#include <Wire.h>
#include <VL53L0X.h>

// ─────────────────────────────────────────────
// PINS
// ─────────────────────────────────────────────
#define MOTOR_IN1   PB1
#define MOTOR_IN2   PB0
#define SERVO_PIN   PA8
#define START_BTN   PC13

// UART to Raspberry Pi
// PA9  = TX (connect to RPi RX)
// PA10 = RX (connect to RPi TX)
// Use Serial1 on STM32

// ─────────────────────────────────────────────
// TCA9548A I2C MULTIPLEXER
// ─────────────────────────────────────────────
#define TCA_ADDR  0x70
#define CH_FRONT  4
#define CH_RIGHT  2
#define CH_LEFT   3
#define CH_COLOR  5

// ─────────────────────────────────────────────
// SERVO SETTINGS
// ─────────────────────────────────────────────
#define SERVO_MIN     35
#define SERVO_CENTER  75
#define SERVO_MAX     130
#define SERVO_STEP    3

// ─────────────────────────────────────────────
// MOTOR SPEEDS
// ─────────────────────────────────────────────
#define SPEED_NORMAL   200
#define SPEED_TURN     170
#define SPEED_REVERSE  180

// ─────────────────────────────────────────────
// DISTANCE THRESHOLDS (mm)
// ─────────────────────────────────────────────
#define FRONT_CRITICAL  120
#define FRONT_CLEAR     200
#define WALL_TARGET     200  // ideal side wall distance

// ─────────────────────────────────────────────
// LAP COUNTING (color sensor)
// ─────────────────────────────────────────────
#define TOTAL_LAPS        3
#define COLOR_THRESHOLD   800   // clear channel threshold — tune this
#define LAP_COOLDOWN      3000  // ms — ignore re-detection for 3 seconds

// ─────────────────────────────────────────────
// STATE MACHINE
// ─────────────────────────────────────────────
enum DriveState {
  S_WAITING,
  S_NORMAL,
  S_REVERSING,
  S_ESCAPE,
  S_FINISHED
};

DriveState state = S_WAITING;

// ─────────────────────────────────────────────
// DIRECTION
// ─────────────────────────────────────────────
// 1 = clockwise, -1 = counter-clockwise
// Set by Raspberry Pi via UART command
int direction = 1;

// ─────────────────────────────────────────────
// OBJECTS
// ─────────────────────────────────────────────
Servo steering;

VL53L0X tofFront;
VL53L0X tofRight;
VL53L0X tofLeft;

// ─────────────────────────────────────────────
// SENSOR DATA
// ─────────────────────────────────────────────
float frontMm = 999;
float rightMm = 999;
float leftMm  = 999;

// Raw color sensor values (read via I2C directly)
uint16_t colorClear = 0;
uint16_t colorR     = 0;
uint16_t colorG     = 0;
uint16_t colorB     = 0;

// ─────────────────────────────────────────────
// LAP TRACKING
// ─────────────────────────────────────────────
int lapCount          = 0;
unsigned long lastLap = 0;
bool onLine           = false;

// ─────────────────────────────────────────────
// SERVO
// ─────────────────────────────────────────────
int targetAngle  = SERVO_CENTER;
int currentAngle = SERVO_CENTER;

// ─────────────────────────────────────────────
// TIMERS
// ─────────────────────────────────────────────
unsigned long tTOF        = 0;
unsigned long tColor      = 0;
unsigned long tCtrl       = 0;
unsigned long tPrint      = 0;
unsigned long tUART       = 0;
unsigned long escapeStart = 0;

// ─────────────────────────────────────────────
// TCA SELECT
// ─────────────────────────────────────────────
void tcaSelect(uint8_t ch) {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
  delayMicroseconds(50);
}

// ─────────────────────────────────────────────
// MOTOR
// ─────────────────────────────────────────────
void motorSet(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    analogWrite(MOTOR_IN1, speed);
    analogWrite(MOTOR_IN2, 0);
  } else if (speed < 0) {
    analogWrite(MOTOR_IN1, 0);
    analogWrite(MOTOR_IN2, -speed);
  } else {
    analogWrite(MOTOR_IN1, 0);
    analogWrite(MOTOR_IN2, 0);
  }
}

// ─────────────────────────────────────────────
// SERVO
// ─────────────────────────────────────────────
void servoSet(int angle) {
  targetAngle = constrain(angle, SERVO_MIN, SERVO_MAX);
}

void servoUpdate() {
  if      (currentAngle < targetAngle) currentAngle += SERVO_STEP;
  else if (currentAngle > targetAngle) currentAngle -= SERVO_STEP;
  steering.write(currentAngle);
}

// ─────────────────────────────────────────────
// TOF INIT
// ─────────────────────────────────────────────
bool initTOF(VL53L0X &sensor, uint8_t ch) {
  tcaSelect(ch);
  sensor.setBus(&Wire);
  if (!sensor.init()) return false;
  sensor.setTimeout(500);
  sensor.startContinuous();
  return true;
}

// ─────────────────────────────────────────────
// TOF READ
// ─────────────────────────────────────────────
float readTOF(VL53L0X &sensor, uint8_t ch) {
  tcaSelect(ch);
  float d = sensor.readRangeContinuousMillimeters();
  if (sensor.timeoutOccurred()) return -1;
  return d;
}

void updateTOF() {
  frontMm = readTOF(tofFront, CH_FRONT);
  rightMm = readTOF(tofRight, CH_RIGHT);
  leftMm  = readTOF(tofLeft,  CH_LEFT);
}

// ─────────────────────────────────────────────
// COLOR SENSOR — raw I2C (no library)
// TCS34725 default address = 0x29
// ─────────────────────────────────────────────
void initColorSensor() {
  tcaSelect(CH_COLOR);
  // Enable power + RGBC
  Wire.beginTransmission(0x29);
  Wire.write(0x80 | 0x00); // ENABLE register
  Wire.write(0x03);         // PON + AEN
  Wire.endTransmission();
  delay(50);
  // Set integration time = 50ms (0xEB)
  Wire.beginTransmission(0x29);
  Wire.write(0x80 | 0x01);
  Wire.write(0xEB);
  Wire.endTransmission();
  // Set gain = 4x (0x01)
  Wire.beginTransmission(0x29);
  Wire.write(0x80 | 0x0F);
  Wire.write(0x01);
  Wire.endTransmission();
}

void updateColor() {
  tcaSelect(CH_COLOR);
  Wire.beginTransmission(0x29);
  Wire.write(0x80 | 0x14); // CDATAL — auto increment
  Wire.endTransmission();
  Wire.requestFrom(0x29, 8);
  if (Wire.available() >= 8) {
    uint8_t buf[8];
    for (int i = 0; i < 8; i++) buf[i] = Wire.read();
    colorClear = buf[0] | (buf[1] << 8);
    colorR     = buf[2] | (buf[3] << 8);
    colorG     = buf[4] | (buf[5] << 8);
    colorB     = buf[6] | (buf[7] << 8);
  }
}

// ─────────────────────────────────────────────
// LAP DETECTION
// Detects orange line (start/finish)
// Orange = high R, high G, low B
// ─────────────────────────────────────────────
bool isOrangeLine() {
  if (colorClear < 300) return false; // too dark, ignore
  // Orange: R dominant, G medium, B low
  return (colorR > colorB * 2) && (colorG > colorB) && (colorB < 400);
}

bool isBlueLine() {
  if (colorClear < 300) return false;
  // Blue: B dominant
  return (colorB > colorR * 2) && (colorB > colorG * 1.5);
}

void checkLap() {
  unsigned long now = millis();
  if (now - lastLap < LAP_COOLDOWN) return;

  bool lineDetected = isOrangeLine() || isBlueLine();

  if (lineDetected && !onLine) {
    onLine = true;
    lapCount++;
    lastLap = now;

    // Tell Raspberry Pi a lap was counted
    Serial1.print(F("LAP:"));
    Serial1.println(lapCount);

    if (lapCount >= TOTAL_LAPS) {
      state = S_FINISHED;
    }
  }

  if (!lineDetected) {
    onLine = false;
  }
}

// ─────────────────────────────────────────────
// STEERING LOGIC
// Stays centered between walls
// direction: 1 = CW, -1 = CCW
// ─────────────────────────────────────────────
int computeSteering() {
  if (rightMm < 0 && leftMm < 0) return SERVO_CENTER;

  // If one sensor fails, steer away from valid wall
  if (rightMm < 0) return SERVO_CENTER + (10 * direction);
  if (leftMm  < 0) return SERVO_CENTER - (10 * direction);

  float error = rightMm - leftMm;

  // Scale factor — tune if steering too aggressive
  float kP = 0.5;

  int angle = SERVO_CENTER - (int)(error * kP);
  return constrain(angle, SERVO_MIN, SERVO_MAX);
}

// ─────────────────────────────────────────────
// UART — receive commands from Raspberry Pi
// Commands:
//   "GO:CW\n"   = start clockwise
//   "GO:CCW\n"  = start counter-clockwise
//   "STOP\n"    = emergency stop
// ─────────────────────────────────────────────
void handleUART() {
  if (Serial1.available()) {
    String cmd = Serial1.readStringUntil('\n');
    cmd.trim();

    if (cmd == F("GO:CW")) {
      direction = 1;
      state = S_NORMAL;
      Serial1.println(F("ACK:GO:CW"));
    }
    else if (cmd == F("GO:CCW")) {
      direction = -1;
      state = S_NORMAL;
      Serial1.println(F("ACK:GO:CCW"));
    }
    else if (cmd == F("STOP")) {
      state = S_FINISHED;
      Serial1.println(F("ACK:STOP"));
    }
  }
}

// ─────────────────────────────────────────────
// SETUP
// ─────────────────────────────────────────────
void setup() {
  Serial.begin(115200);   // USB debug
  Serial1.begin(115200);  // UART to Raspberry Pi (PA9/PA10)

  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(START_BTN, INPUT_PULLUP);

  steering.attach(SERVO_PIN);
  steering.write(SERVO_CENTER);

  Wire.begin();
  Wire.setClock(400000);

  bool f = initTOF(tofFront, CH_FRONT);
  bool r = initTOF(tofRight, CH_RIGHT);
  bool l = initTOF(tofLeft,  CH_LEFT);
  initColorSensor();

  Serial.print(F("TOF Front:"));  Serial.println(f ? F("OK") : F("FAIL"));
  Serial.print(F("TOF Right:"));  Serial.println(r ? F("OK") : F("FAIL"));
  Serial.print(F("TOF Left:"));   Serial.println(l ? F("OK") : F("FAIL"));
  Serial.println(F("READY — Waiting for button or UART GO command"));
}

// ─────────────────────────────────────────────
// LOOP
// ─────────────────────────────────────────────
void loop() {
  unsigned long now = millis();

  servoUpdate();
  handleUART();

  // ── BUTTON START (manual fallback) ──
  if (state == S_WAITING && digitalRead(START_BTN) == LOW) {
    delay(50);
    if (digitalRead(START_BTN) == LOW) {
      direction = 1; // default clockwise
      state = S_NORMAL;
      Serial1.println(F("STARTED:MANUAL"));
    }
  }

  // ── SENSOR UPDATES ──
  if (now - tTOF > 30) {
    tTOF = now;
    updateTOF();
  }

  if (now - tColor > 80) {
    tColor = now;
    updateColor();
    if (state == S_NORMAL) checkLap();
  }

  // ── CONTROL LOOP ──
  if (now - tCtrl > 10) {
    tCtrl = now;

    switch (state) {

      case S_NORMAL: {
        // Obstacle ahead — reverse
        if (frontMm > 0 && frontMm < FRONT_CRITICAL) {
          state = S_REVERSING;
          break;
        }

        // Steer to stay centered
        servoSet(computeSteering());

        // Slow down near walls
        int spd = (frontMm > 0 && frontMm < 300) ? SPEED_TURN : SPEED_NORMAL;
        motorSet(spd);
        break;
      }

      case S_REVERSING: {
        motorSet(-SPEED_REVERSE);
        // Steer opposite to direction of travel
        servoSet(direction == 1 ? SERVO_MAX : SERVO_MIN);

        if (frontMm < 0 || frontMm > FRONT_CLEAR) {
          state = S_ESCAPE;
          escapeStart = now;
        }
        break;
      }

      case S_ESCAPE: {
        motorSet(SPEED_TURN);
        servoSet(direction == 1 ? SERVO_MIN : SERVO_MAX);

        if (now - escapeStart > 700) {
          state = S_NORMAL;
          servoSet(SERVO_CENTER);
        }
        break;
      }

      case S_FINISHED: {
        motorSet(0);
        servoSet(SERVO_CENTER);
        Serial1.println(F("DONE"));
        break;
      }

      case S_WAITING:
      default:
        motorSet(0);
        break;
    }
  }

  // ── DEBUG PRINT ──
  if (now - tPrint > 300) {
    tPrint = now;
    Serial.print(F("STATE:"));  Serial.print(state);
    Serial.print(F(" F:"));     Serial.print(frontMm);
    Serial.print(F(" R:"));     Serial.print(rightMm);
    Serial.print(F(" L:"));     Serial.print(leftMm);
    Serial.print(F(" LAP:"));   Serial.print(lapCount);
    Serial.print(F(" C:"));     Serial.print(colorClear);
    Serial.print(F(" R:"));     Serial.print(colorR);
    Serial.print(F(" G:"));     Serial.print(colorG);
    Serial.print(F(" B:"));     Serial.println(colorB);
  }
}