#include <QTRSensors.h>
#include <Servo.h>

// ---------------- QTR ----------------
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
uint8_t qtrPins[SensorCount] = {A0, A1, A2, A3, A4, A5, A6, A7};

// ---------------- MOTOR PINS ----------------
#define FL_RPWM 5
#define FL_LPWM 6
#define FR_RPWM 7
#define FR_LPWM 8
#define BL_RPWM 9
#define BL_LPWM 10
#define BR_RPWM 11
#define BR_LPWM 12

// ---------------- PID ----------------
float Kp = 0.05;
float Kd = 0.10;

int baseSpeed = 104;
int maxSpeed  = 240;
int turnSpeed = 105;

int lastError = 0;

int frReverseBoost = 40;
int leftTurnFRBoost = 45;
int rightTurnFRBoost = 45;

// ---------------- TCRT ----------------
#define TCRT_PIN 18
int junctionCount = 0;
bool tcrtState = false;
bool tcrtLastState = false;
unsigned long tcrtBlockUntil = 0;

// ---------------- TURN SETTINGS ----------------
const unsigned long moveStraightBeforeTurn = 120;
const unsigned long turnDuration = 620;

// ---------------- POST S2 LINE ACQUIRE SETTINGS ----------------
const int POST_S2_LINE_DETECT_THRESHOLD = 650;
const int POST_S2_ACQUIRE_SPIN_SPEED = 85;
const int POST_S2_CENTERING_SPEED = 60;

// ---------------- MAIN COLOR SENSOR ----------------
#define S0 30
#define S1 31
#define S2 32
#define S3 33
#define OUT 34

long redFreq, greenFreq, blueFreq;

// ---------------- PERIPHERAL COLOR SENSORS ----------------
#define SHARED_S0 35
#define SHARED_S1 36

#define P1_S2 37
#define P1_S3 38
#define P1_OUT 43

#define P2_S2 39
#define P2_S3 40
#define P2_OUT 44

#define P3_S2 41
#define P3_S3 42
#define P3_OUT 45

#define P4_S2 48
#define P4_S3 49
#define P4_OUT 47

// ---------------- FIRING ----------------
Servo servoX;
Servo servoY;

#define SERVO_X_PIN 4
#define SERVO_Y_PIN 3
#define RELAY_PIN 46

const int RELAY_ON  = LOW;
const int RELAY_OFF = HIGH;

const int P1_X = 145;
const int P1_Y = 90;

const int P2_X = 60;
const int P2_Y = 100;

const int P3_X = 150;
const int P3_Y = 145;

const int P4_X = 75;
const int P4_Y = 165;

// Servo initial/home position
const int SERVO_HOME_X = 120;
const int SERVO_HOME_Y = 125;

int currentX = SERVO_HOME_X;
int currentY = SERVO_HOME_Y;

typedef enum {
  COLOR_UNKNOWN,
  COLOR_RED,
  COLOR_BLUE
} DetectedColor;

// ---------------- ROUTE STATES ----------------
enum RouteState {
  GO_TO_J4,
  GO_TO_J7,
  GO_TO_J10,

  PHASE1_BLUE_TO_1,
  PHASE1_BLUE_TO_2,
  PHASE1_BLUE_TO_FINAL_1,

  PHASE1_WHITE_TO_2_A,
  PHASE1_WHITE_TO_2_B,

  PHASE2_BLUE_TO_2,
  PHASE2_BLUE_TO_1,

  PHASE3_TO_1,
  PHASE3_TO_2,
  PHASE3_BLUE_EXIT_2,

  PHASE4_TO_1,
  PHASE4_TO_2,

  PHASE4_EXIT_TO_1,
  PHASE4_EXIT_TO_2,
  PHASE4_EXIT_TO_3,
  PHASE4_EXIT_TO_4,

  S2_GO_2_JUNCTIONS,
  S2_SLOW_TO_NEXT_JUNCTION,
  S2_DONE,

  POST_S2_GO_2_JUNCTIONS,
  POST_S2_GO_1_JUNCTION,
  POST_S2_GAP_MODE,
  POST_S2_FIND_JUNCTION_AFTER_RAMP,

  FINAL_STOP
};

RouteState state = GO_TO_J4;
int stateStartJunction = 0;
bool stoppedForever = false;

// ---------------- MOTOR FUNCTIONS ----------------
void setLeftMotor(int speedVal) {
  speedVal = constrain(speedVal, -maxSpeed, maxSpeed);

  if (speedVal >= 0) {
    analogWrite(FL_RPWM, speedVal);
    analogWrite(FL_LPWM, 0);
    analogWrite(BL_RPWM, speedVal);
    analogWrite(BL_LPWM, 0);
  } else {
    analogWrite(FL_RPWM, 0);
    analogWrite(FL_LPWM, -speedVal);
    analogWrite(BL_RPWM, 0);
    analogWrite(BL_LPWM, -speedVal);
  }
}

void setRightMotor(int speedVal) {
  speedVal = constrain(speedVal, -maxSpeed, maxSpeed);

  if (speedVal >= 0) {
    analogWrite(FR_RPWM, speedVal);
    analogWrite(FR_LPWM, 0);
    analogWrite(BR_RPWM, speedVal);
    analogWrite(BR_LPWM, 0);
  } else {
    analogWrite(FR_RPWM, 0);
    analogWrite(FR_LPWM, -speedVal);
    analogWrite(BR_RPWM, 0);
    analogWrite(BR_LPWM, -speedVal);
  }
}

void setMotor(int leftSpeed, int rightSpeed) {
  setLeftMotor(leftSpeed);
  setRightMotor(rightSpeed);
}

void stopRobot() {
  setMotor(0, 0);
}

void stopForever() {
  stopRobot();
  stoppedForever = true;
  while (true) {
    stopRobot();
  }
}

// ---------------- SPEED HELPERS ----------------
int blueExitSpeed() {
  return baseSpeed * 0.7;
}

// ---------------- LINE FOLLOWING ----------------
void followLinePID(int speedUse = -1) {
  if (speedUse < 0) speedUse = baseSpeed;

  uint16_t position = qtr.readLineBlack(sensorValues);
  int error = (int)position - 3500;
  int derivative = error - lastError;

  int correction = (Kp * error) + (Kd * derivative);
  correction = constrain(correction, -70, 70);

  int leftSpeed  = speedUse + correction;
  int rightSpeed = speedUse - correction;

  leftSpeed  = constrain(leftSpeed, 0, maxSpeed);
  rightSpeed = constrain(rightSpeed, 0, maxSpeed);

  setMotor(leftSpeed, rightSpeed);
  lastError = error;
}

bool lineSeen() {
  qtr.read(sensorValues);
  for (int i = 0; i < SensorCount; i++) {
    if (sensorValues[i] > 900) return true;
  }
  return false;
}

void reacquireLine(int dir) {
  unsigned long t0 = millis();

  while (millis() - t0 < 700) {
    if (lineSeen()) break;
    setMotor(70 * dir, -70 * dir);
  }

  stopRobot();
  delay(30);
}

// ---------------- POST S2 LINE ACQUIRE ----------------
bool postS2LineSeenStrong() {
  qtr.readCalibrated(sensorValues);

  int seenCount = 0;
  for (int i = 0; i < SensorCount; i++) {
    if (sensorValues[i] > POST_S2_LINE_DETECT_THRESHOLD) {
      seenCount++;
    }
  }

  return seenCount >= 1;
}

bool postS2LineCentered() {
  qtr.readCalibrated(sensorValues);

  bool leftCenter  = sensorValues[3] > POST_S2_LINE_DETECT_THRESHOLD;
  bool rightCenter = sensorValues[4] > POST_S2_LINE_DETECT_THRESHOLD;

  return leftCenter || rightCenter;
}

void postS2StrongAcquireLine(int preferredDir) {
  stopRobot();
  delay(60);

  unsigned long t0;

  t0 = millis();
  while (millis() - t0 < 900) {
    if (postS2LineSeenStrong()) break;
    setMotor(POST_S2_ACQUIRE_SPIN_SPEED * preferredDir,
             -POST_S2_ACQUIRE_SPIN_SPEED * preferredDir);
    delay(5);
  }

  if (!postS2LineSeenStrong()) {
    t0 = millis();
    while (millis() - t0 < 1400) {
      if (postS2LineSeenStrong()) break;
      setMotor(-POST_S2_ACQUIRE_SPIN_SPEED * preferredDir,
                POST_S2_ACQUIRE_SPIN_SPEED * preferredDir);
      delay(5);
    }
  }

  if (!postS2LineSeenStrong()) {
    t0 = millis();
    while (millis() - t0 < 500) {
      if (postS2LineSeenStrong()) break;
      setMotor(65, 65);
      delay(5);
    }
  }

  t0 = millis();
  while (millis() - t0 < 900) {
    if (postS2LineCentered()) break;

    uint16_t pos = qtr.readLineBlack(sensorValues);

    if (pos < 3500) {
      setMotor(-POST_S2_CENTERING_SPEED, POST_S2_CENTERING_SPEED);
    } else {
      setMotor(POST_S2_CENTERING_SPEED, -POST_S2_CENTERING_SPEED);
    }

    delay(5);
  }

  stopRobot();
  delay(80);
  lastError = 0;
}

// ---------------- MOVEMENT HELPERS ----------------
void reverseBrief() {
  int leftRev  = -baseSpeed;
  int rightRev = -baseSpeed - frReverseBoost;

  setMotor(leftRev, rightRev);
  delay(230);
  stopRobot();
  delay(80);
}

void reverseBeforeFiring() {
  stopRobot();
  delay(100);

  setMotor(-90, -110);
  delay(130 );

  stopRobot();
  delay(150);
}

void reverseTinyForPhase3Blue() {
  setMotor(-110, -140);
  delay(220);
  stopRobot();
  delay(90);
}

void reverseStrongForPhase4() {
  setMotor(-140, -180);
  delay(310);
  stopRobot();
  delay(100);
}

void turnRobot(int direction, bool onSpot = false, unsigned long turnMs = turnDuration) {
  tcrtBlockUntil = millis() + 900;

  if (!onSpot) {
    setMotor(baseSpeed, baseSpeed);
    delay(moveStraightBeforeTurn);
  }

  unsigned long t0 = millis();

  while (millis() - t0 < turnMs) {
    int left  = turnSpeed * direction;
    int right = -turnSpeed * direction;

    if (direction == -1) right += leftTurnFRBoost;
    if (direction == 1)  right -= rightTurnFRBoost;

    setMotor(left, right);
  }

  stopRobot();
  delay(50);

  lastError = 0;
  reacquireLine(direction);

  tcrtLastState = digitalRead(TCRT_PIN);
  tcrtBlockUntil = millis() + 150;
}

void turnRight() {
  turnRobot(1, false);
}

void turnLeft() {
  turnRobot(-1, false);
}

void turnLeftOnSpot() {
  turnRobot(-1, true);
}

// ---------------- POST S2 MOVEMENT HELPERS ----------------
void postS2ReverseStart() {
  setMotor(-baseSpeed, -baseSpeed - frReverseBoost);
  delay(387);
  stopRobot();
  delay(120);
}

void postS2TurnRobot(int direction, bool onSpot = false, unsigned long turnMs = turnDuration) {
  tcrtBlockUntil = millis() + 1000;

  if (!onSpot) {
    setMotor(baseSpeed, baseSpeed);
    delay(moveStraightBeforeTurn);
  }

  unsigned long t0 = millis();

  while (millis() - t0 < turnMs) {
    int left  = turnSpeed * direction;
    int right = -turnSpeed * direction;

    if (direction == -1) right += leftTurnFRBoost;
    if (direction == 1)  right -= rightTurnFRBoost;

    setMotor(left, right);
    delay(5);
  }

  stopRobot();
  delay(70);

  postS2StrongAcquireLine(direction);

  tcrtLastState = digitalRead(TCRT_PIN);
  tcrtBlockUntil = millis() + 250;
}

void postS2TurnRight() {
  postS2TurnRobot(1, false);
}

void postS2TurnLeftOnSpotNoAcquire() {
  tcrtBlockUntil = millis() + 1000;

  unsigned long t0 = millis();

  while (millis() - t0 < turnDuration) {
    setMotor(-turnSpeed, turnSpeed);
    delay(5);
  }

  stopRobot();
  delay(100);

  lastError = 0;
  tcrtLastState = digitalRead(TCRT_PIN);
  tcrtBlockUntil = millis() + 300;
}

void postS2GapModeStraight() {
  setMotor(baseSpeed, baseSpeed);
}

void postS2RampMode() {
  stopRobot();
  delay(150);

  // smooth start, less current spike
  setMotor(200,200 );
  //delay(250);

  //setMotor(170, 170);
  delay(1220);

  stopRobot();
  delay(100);

  tcrtLastState = digitalRead(TCRT_PIN);
  tcrtBlockUntil = millis() + 300;
}

// ---------------- MAIN COLOR SENSOR ----------------
long readColor(int s2, int s3) {
  digitalWrite(S2, s2);
  digitalWrite(S3, s3);
  delay(3);
  return pulseIn(OUT, LOW, 30000);
}

int detectMainColor() {
  redFreq   = readColor(LOW, LOW);
  greenFreq = readColor(HIGH, HIGH);
  blueFreq  = readColor(LOW, HIGH);

  long maxVal = max(redFreq, max(greenFreq, blueFreq));
  long minVal = min(redFreq, min(greenFreq, blueFreq));

  if ((maxVal - minVal) < 90) return 1; // WHITE

  float redDiff   = (redFreq - blueFreq) / (float)redFreq;
  float greenDiff = (greenFreq - blueFreq) / (float)greenFreq;

  if (redDiff > 0.15 && greenDiff > 0.15) return 2; // BLUE

  return 0;
}

int detectColorStable(unsigned long ms = 350) {
  int blueCount = 0;
  int whiteCount = 0;

  unsigned long t0 = millis();

  while (millis() - t0 < ms) {
    int c = detectMainColor();

    if (c == 2) blueCount++;
    if (c == 1) whiteCount++;

    delay(40);
  }

  if (blueCount >= whiteCount && blueCount > 0) return 2;
  if (whiteCount > 0) return 1;

  return 0;
}

// ---------------- PERIPHERAL FIRING COLOR SENSOR ----------------
long readSensorColor(int s2Pin, int s3Pin, int outPin, int s2State, int s3State) {
  digitalWrite(s2Pin, s2State);
  digitalWrite(s3Pin, s3State);
  delay(5);
  return pulseIn(outPin, LOW, 30000);
}

void readRGB(int s2Pin, int s3Pin, int outPin, long &r, long &g, long &b) {
  r = readSensorColor(s2Pin, s3Pin, outPin, LOW, LOW);
  g = readSensorColor(s2Pin, s3Pin, outPin, HIGH, HIGH);
  b = readSensorColor(s2Pin, s3Pin, outPin, LOW, HIGH);
}

DetectedColor detectPeripheralTargetColor(int s2Pin, int s3Pin, int outPin) {
  long r, g, b;
  readRGB(s2Pin, s3Pin, outPin, r, g, b);

  if (r == 0 || g == 0 || b == 0) return COLOR_UNKNOWN;

  if (b < r * 0.85 && b < g * 0.85) return COLOR_BLUE;
  if (r < g * 0.85 && r < b * 0.85) return COLOR_RED;

  return COLOR_UNKNOWN;
}

void applyTwoBlueTwoRedRule(DetectedColor color[4]) {
  int blueCount = 0;
  int redCount = 0;
  int unknownCount = 0;
  int unknownIndex = -1;

  for (int i = 0; i < 4; i++) {
    if (color[i] == COLOR_BLUE) blueCount++;
    else if (color[i] == COLOR_RED) redCount++;
    else {
      unknownCount++;
      unknownIndex = i;
    }
  }

  if (unknownCount == 1) {
    if (blueCount == 2 && redCount == 1) {
      color[unknownIndex] = COLOR_RED;
    } else if (blueCount == 1 && redCount == 2) {
      color[unknownIndex] = COLOR_BLUE;
    }
  }
}

// ---------------- SERVO MOVEMENT ----------------
void moveToCoordinate(int x, int y) {
  int dx = x - currentX;
  int dy = y - currentY;

  int steps = max(abs(dx), abs(dy));

  if (steps == 0) {
    servoX.write(x);
    servoY.write(y);
    delay(200);
    return;
  }

  for (int i = 1; i <= steps; i++) {
    int nextX = currentX + (dx * i) / steps;
    int nextY = currentY + (dy * i) / steps;

    servoX.write(nextX);
    servoY.write(nextY);
    delay(12);
  }

  currentX = x;
  currentY = y;

  delay(250);
}

void returnServosHome() {
  moveToCoordinate(SERVO_HOME_X, SERVO_HOME_Y);
}

void fireOnce() {
  digitalWrite(RELAY_PIN, RELAY_ON);
  delay(120);
  digitalWrite(RELAY_PIN, RELAY_OFF);
  delay(400);
}

void aimAndFire(const char* name, int x, int y) {
  Serial.print("AIMING AT ");
  Serial.println(name);

  moveToCoordinate(x, y);

  Serial.print("FIRING AT ");
  Serial.println(name);

  fireOnce();
}

void fireAtDetectedBlueTargets() {
  DetectedColor color[4];

  // Detect ALL colors first
  color[0] = detectPeripheralTargetColor(P1_S2, P1_S3, P1_OUT);
  color[1] = detectPeripheralTargetColor(P2_S2, P2_S3, P2_OUT);
  color[2] = detectPeripheralTargetColor(P3_S2, P3_S3, P3_OUT);
  color[3] = detectPeripheralTargetColor(P4_S2, P4_S3, P4_OUT);

  // Apply red/blue correction rule before firing
  applyTwoBlueTwoRedRule(color);

  // Do not fire immediately after detecting.
  // First reverse a little bit.
  reverseBeforeFiring();

  // Fire only BLUE targets
  if (color[0] == COLOR_BLUE) aimAndFire("P1", P1_X, P1_Y);
  if (color[1] == COLOR_BLUE) aimAndFire("P2", P2_X, P2_Y);
  if (color[2] == COLOR_BLUE) aimAndFire("P3", P3_X, P3_Y);
  if (color[3] == COLOR_BLUE) aimAndFire("P4", P4_X, P4_Y);

  // After every firing section, return servos to initial position
  returnServosHome();
}

void fireS1_Phase1() {
  fireAtDetectedBlueTargets();
}

void fireS1_Phase2() {
  fireAtDetectedBlueTargets();
}

void fireS2() {
  fireAtDetectedBlueTargets();
}

// ---------------- SMALL FORWARD HELPERS ----------------
void smallForwardForPhase2Color() {
  unsigned long t0 = millis();

  while (millis() - t0 < 780) {
    followLinePID(50);
    delay(8);
  }

  stopRobot();
  delay(80);
}

void smallForwardS2Final() {
  unsigned long t0 = millis();

  while (millis() - t0 < 1250) {
    followLinePID(baseSpeed / 2);
    delay(8);
  }

  stopRobot();
  delay(150);
}
void detectFinalColorsFor6Sec(DetectedColor color[4]) {
  int blueCount[4] = {0, 0, 0, 0};
  int redCount[4]  = {0, 0, 0, 0};

  unsigned long t0 = millis();

  while (millis() - t0 < 6000) {
    DetectedColor c[4];

    c[0] = detectPeripheralTargetColor(P1_S2, P1_S3, P1_OUT);
    c[1] = detectPeripheralTargetColor(P2_S2, P2_S3, P2_OUT);
    c[2] = detectPeripheralTargetColor(P3_S2, P3_S3, P3_OUT);
    c[3] = detectPeripheralTargetColor(P4_S2, P4_S3, P4_OUT);

    for (int i = 0; i < 4; i++) {
      if (c[i] == COLOR_BLUE) blueCount[i]++;
      else if (c[i] == COLOR_RED) redCount[i]++;
    }

    delay(80);
  }

  for (int i = 0; i < 4; i++) {
    if (blueCount[i] > redCount[i] && blueCount[i] > 0) {
      color[i] = COLOR_BLUE;
    } else if (redCount[i] > blueCount[i] && redCount[i] > 0) {
      color[i] = COLOR_RED;
    } else {
      color[i] = COLOR_UNKNOWN;
    }
  }

  applyTwoBlueTwoRedRule(color);
}

void finalPostRampFireNoReverse() {
  DetectedColor color[4];

  stopRobot();
  delay(150);

  detectFinalColorsFor6Sec(color);

  if (color[0] == COLOR_BLUE) aimAndFire("P1", P1_X, P1_Y);
  if (color[1] == COLOR_BLUE) aimAndFire("P2", P2_X, P2_Y);
  if (color[2] == COLOR_BLUE) aimAndFire("P3", P3_X, P3_Y);
  if (color[3] == COLOR_BLUE) aimAndFire("P4", P4_X, P4_Y);

  returnServosHome();
}

void reverseTwoJunctionsAndStop() {
  stopRobot();
  delay(200);

  int reverseJunctions = 0;
  bool last = digitalRead(TCRT_PIN);

  tcrtBlockUntil = millis() + 300;

  while (reverseJunctions < 2) {
    setMotor(-baseSpeed, -baseSpeed - frReverseBoost);

    bool now = digitalRead(TCRT_PIN);

    if (millis() > tcrtBlockUntil && now && !last) {
      reverseJunctions++;

      Serial.print("Reverse Junction: ");
      Serial.println(reverseJunctions);

      tcrtBlockUntil = millis() + 350;
    }

    last = now;
    delay(5);
  }

  stopForever();
}
void postS2FinalForwardSlow() {
  unsigned long t0 = millis();

  while (millis() - t0 < 1600) {
    followLinePID(baseSpeed * 0.44);
    delay(8);
  }

  stopRobot();
  delay(400);

  finalPostRampFireNoReverse();

  delay(400);

  reverseTwoJunctionsAndStop();
}

// ---------------- STATE START HELPERS ----------------
void startS2Path() {
  stopRobot();
  delay(120);

  tcrtLastState = digitalRead(TCRT_PIN);
  tcrtBlockUntil = millis() + 250;

  state = S2_GO_2_JUNCTIONS;
  stateStartJunction = junctionCount;
}

void startPostS2Work() {
  postS2ReverseStart();
  postS2TurnRight();

  state = POST_S2_GO_2_JUNCTIONS;
  stateStartJunction = junctionCount;

  tcrtLastState = digitalRead(TCRT_PIN);
  tcrtBlockUntil = millis() + 300;
}

void goToS1Phase3() {
  reverseBrief();
  turnLeftOnSpot();

  tcrtLastState = digitalRead(TCRT_PIN);
  tcrtBlockUntil = millis() + 150;

  state = PHASE3_TO_1;
  stateStartJunction = junctionCount;
}

void goToS1Phase4() {
  reverseBrief();
  turnLeftOnSpot();

  tcrtLastState = digitalRead(TCRT_PIN);
  tcrtBlockUntil = millis() + 150;

  state = PHASE4_TO_1;
  stateStartJunction = junctionCount;
}

// ---------------- ROUTE HANDLER ----------------
void handleJunction() {
  int passed = junctionCount - stateStartJunction;

  Serial.print("State: ");
  Serial.print(state);
  Serial.print("  Passed: ");
  Serial.println(passed);

  switch (state) {
    case GO_TO_J4:
      if (junctionCount >= 4) {
        turnRight();
        state = GO_TO_J7;
        stateStartJunction = junctionCount;
      }
      break;

    case GO_TO_J7:
      if (junctionCount >= 7) {
        turnRight();
        state = GO_TO_J10;
        stateStartJunction = junctionCount;
      }
      break;

    case GO_TO_J10:
      if (passed >= 1) {
        delay(250);
        stopRobot();
        delay(250);

        int color = detectColorStable(400);

        if (color == 2) {
          fireS1_Phase1();

          reverseBrief();
          turnLeftOnSpot();

          tcrtLastState = digitalRead(TCRT_PIN);
          tcrtBlockUntil = millis() + 150;

          state = PHASE1_BLUE_TO_1;
          stateStartJunction = junctionCount;
        }

        else if (color == 1) {
          reverseBrief();
          turnLeftOnSpot();

          tcrtLastState = digitalRead(TCRT_PIN);
          tcrtBlockUntil = millis() + 150;

          state = PHASE1_WHITE_TO_2_A;
          stateStartJunction = junctionCount;
        }

        else {
          stopForever();
        }
      }
      break;

    case PHASE1_BLUE_TO_1:
      if (passed >= 1) {
        stopRobot();
        delay(80);

        turnRight();

        state = PHASE1_BLUE_TO_2;
        stateStartJunction = junctionCount;

        tcrtLastState = digitalRead(TCRT_PIN);
        tcrtBlockUntil = millis() + 150;
      }
      break;

    case PHASE1_BLUE_TO_2:
      if (passed >= 2) {
        stopRobot();
        delay(80);

        turnLeft();

        state = PHASE1_BLUE_TO_FINAL_1;
        stateStartJunction = junctionCount;

        tcrtLastState = digitalRead(TCRT_PIN);
        tcrtBlockUntil = millis() + 150;
      }
      break;

    case PHASE1_BLUE_TO_FINAL_1:
      if (passed >= 1) {
        startS2Path();
      }
      break;

    case PHASE1_WHITE_TO_2_A:
      if (passed >= 1) {
        turnRight();
        state = PHASE1_WHITE_TO_2_B;
        stateStartJunction = junctionCount;
      }
      break;

    case PHASE1_WHITE_TO_2_B:
      if (passed >= 1) {
        turnRight();

        smallForwardForPhase2Color();

        {
          int color = detectColorStable(350);

          if (color == 2) {
            fireS1_Phase2();

            reverseBrief();
            turnLeftOnSpot();

            tcrtLastState = digitalRead(TCRT_PIN);
            tcrtBlockUntil = millis() + 150;

            state = PHASE2_BLUE_TO_2;
            stateStartJunction = junctionCount;
          }

          else if (color == 1) {
            goToS1Phase3();
          }

          else {
            stopForever();
          }
        }
      }
      break;

    case PHASE2_BLUE_TO_2:
      if (passed >= 1) {
        turnLeft();
        state = PHASE2_BLUE_TO_1;
        stateStartJunction = junctionCount;
      }
      break;

    case PHASE2_BLUE_TO_1:
      if (passed >= 1) {
        startS2Path();
      }
      break;

    case PHASE3_TO_1:
      if (passed >= 1) {
        turnRight();

        state = PHASE3_TO_2;
        stateStartJunction = junctionCount;

        tcrtLastState = digitalRead(TCRT_PIN);
        tcrtBlockUntil = millis() + 150;
      }
      break;

    case PHASE3_TO_2:
      if (passed >= 1) {
        turnRight();

        smallForwardForPhase2Color();

        {
          int color = detectColorStable(350);

          if (color == 2) {
            fireS1_Phase2();

            reverseTinyForPhase3Blue();
            turnRight();

            tcrtLastState = digitalRead(TCRT_PIN);
            tcrtBlockUntil = millis() + 150;

            state = PHASE3_BLUE_EXIT_2;
            stateStartJunction = junctionCount;
          }

          else if (color == 1) {
            goToS1Phase4();
          }

          else {
            stopForever();
          }
        }
      }
      break;

    case PHASE3_BLUE_EXIT_2:
      if (passed >= 2) {
        startS2Path();
      }
      break;

    case PHASE4_TO_1:
      if (passed >= 1) {
        turnRight();

        state = PHASE4_TO_2;
        stateStartJunction = junctionCount;

        tcrtLastState = digitalRead(TCRT_PIN);
        tcrtBlockUntil = millis() + 150;
      }
      break;

    case PHASE4_TO_2:
      if (passed >= 1) {
        turnRight();

        smallForwardForPhase2Color();

        {
          int color = detectColorStable(350);

          if (color == 2 || color == 1) {
            fireS1_Phase2();

            reverseStrongForPhase4();
            turnLeft();

            tcrtLastState = digitalRead(TCRT_PIN);
            tcrtBlockUntil = millis() + 150;

            state = PHASE4_EXIT_TO_1;
            stateStartJunction = junctionCount;
          }

          else {
            stopForever();
          }
        }
      }
      break;

    case PHASE4_EXIT_TO_1:
      if (passed >= 1) {
        turnRight();
        state = PHASE4_EXIT_TO_2;
        stateStartJunction = junctionCount;
      }
      break;

    case PHASE4_EXIT_TO_2:
      if (passed >= 2) {
        turnRight();
        state = PHASE4_EXIT_TO_3;
        stateStartJunction = junctionCount;
      }
      break;

    case PHASE4_EXIT_TO_3:
      if (passed >= 2) {
        turnLeft();
        state = PHASE4_EXIT_TO_4;
        stateStartJunction = junctionCount;
      }
      break;

    case PHASE4_EXIT_TO_4:
      if (passed >= 1) {
        startS2Path();
      }
      break;

    case S2_GO_2_JUNCTIONS:
      if (passed >= 2) {
        turnRight();

        tcrtLastState = digitalRead(TCRT_PIN);
        tcrtBlockUntil = millis() + 150;

        state = S2_SLOW_TO_NEXT_JUNCTION;
        stateStartJunction = junctionCount;
      }
      break;

    case S2_SLOW_TO_NEXT_JUNCTION:
      if (passed >= 1) {
        smallForwardS2Final();

        fireS2();

        delay(800);
        startPostS2Work();
      }
      break;

    case POST_S2_GO_2_JUNCTIONS:
      if (passed >= 2) {
        postS2TurnRight();

        state = POST_S2_GO_1_JUNCTION;
        stateStartJunction = junctionCount;
      }
      break;

    case POST_S2_GO_1_JUNCTION:
      if (passed >= 1) {
        stopRobot();
        delay(500);

        state = POST_S2_GAP_MODE;
        stateStartJunction = junctionCount;

        tcrtLastState = digitalRead(TCRT_PIN);
        tcrtBlockUntil = millis() + 250;
      }
      break;

    case POST_S2_GAP_MODE:
      stopRobot();
      delay(100);

      postS2RampMode();

      state = POST_S2_FIND_JUNCTION_AFTER_RAMP;
      stateStartJunction = junctionCount;

      tcrtLastState = digitalRead(TCRT_PIN);
      tcrtBlockUntil = millis() + 250;
      break;

    case POST_S2_FIND_JUNCTION_AFTER_RAMP:
      stopRobot();
      delay(100);

      postS2TurnLeftOnSpotNoAcquire();
      postS2FinalForwardSlow();
      break;

    case S2_DONE:
      stopForever();
      break;

    case FINAL_STOP:
      stopForever();
      break;
  }
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(9600);

  pinMode(FL_RPWM, OUTPUT);
  pinMode(FL_LPWM, OUTPUT);
  pinMode(FR_RPWM, OUTPUT);
  pinMode(FR_LPWM, OUTPUT);
  pinMode(BL_RPWM, OUTPUT);
  pinMode(BL_LPWM, OUTPUT);
  pinMode(BR_RPWM, OUTPUT);
  pinMode(BR_LPWM, OUTPUT);

  pinMode(TCRT_PIN, INPUT);

  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(OUT, INPUT);

  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);

  pinMode(SHARED_S0, OUTPUT);
  pinMode(SHARED_S1, OUTPUT);

  digitalWrite(SHARED_S0, HIGH);
  digitalWrite(SHARED_S1, LOW);

  pinMode(P1_S2, OUTPUT);
  pinMode(P1_S3, OUTPUT);
  pinMode(P1_OUT, INPUT);

  pinMode(P2_S2, OUTPUT);
  pinMode(P2_S3, OUTPUT);
  pinMode(P2_OUT, INPUT);

  pinMode(P3_S2, OUTPUT);
  pinMode(P3_S3, OUTPUT);
  pinMode(P3_OUT, INPUT);

  pinMode(P4_S2, OUTPUT);
  pinMode(P4_S3, OUTPUT);
  pinMode(P4_OUT, INPUT);

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, RELAY_OFF);

  servoX.attach(SERVO_X_PIN);
  servoY.attach(SERVO_Y_PIN);

  servoX.write(SERVO_HOME_X);
  servoY.write(SERVO_HOME_Y);

  qtr.setTypeAnalog();
  qtr.setSensorPins(qtrPins, SensorCount);
  qtr.setEmitterPin(2);

  delay(1000);

  for (int i = 0; i < 200; i++) {
    qtr.calibrate();
    delay(20);
  }

  tcrtLastState = digitalRead(TCRT_PIN);
  tcrtBlockUntil = millis() + 300;

  state = GO_TO_J4;
  stateStartJunction = 0;
}

// ---------------- LOOP ----------------
void loop() {
  if (stoppedForever) {
    stopRobot();
    return;
  }

  tcrtState = digitalRead(TCRT_PIN);

  bool allowTcrt = millis() > tcrtBlockUntil;

  if (allowTcrt && tcrtState && !tcrtLastState) {
    junctionCount++;

    Serial.print("Junction: ");
    Serial.println(junctionCount);

    handleJunction();
  }

  tcrtLastState = tcrtState;

  if (!stoppedForever) {
    if (state == POST_S2_GAP_MODE) {
      postS2GapModeStraight();
    }

    else if (
      state == POST_S2_GO_2_JUNCTIONS ||
      state == POST_S2_GO_1_JUNCTION
    ) {
      followLinePID(baseSpeed * 0.7);
    }

    else if (state == POST_S2_FIND_JUNCTION_AFTER_RAMP) {
      followLinePID(baseSpeed);
    }

    else if (
      state == PHASE1_BLUE_TO_1 ||
      state == PHASE1_BLUE_TO_2 ||
      state == PHASE1_BLUE_TO_FINAL_1 ||
      state == PHASE2_BLUE_TO_2 ||
      state == PHASE2_BLUE_TO_1 ||
      state == PHASE3_BLUE_EXIT_2 ||
      state == PHASE4_EXIT_TO_1 ||
      state == PHASE4_EXIT_TO_2 ||
      state == PHASE4_EXIT_TO_3 ||
      state == PHASE4_EXIT_TO_4 ||
      state == S2_SLOW_TO_NEXT_JUNCTION
    ) {
      followLinePID(blueExitSpeed());
    }

    else {
      followLinePID();
    }

    delay(8);
  }
}
