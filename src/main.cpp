#include <Wire.h>
#include <Arduino.h>
#include "Adafruit_VL6180X.h"
#include <MPU9250_asukiaaa.h>

#define BTN_PIN 14

// ====== I²C and MUX Pins ======
#define SDA_PIN 26
#define SCL_PIN 25
#define MUX_ADDR 0x70

#define BUZZER_PIN 27


// ====== MUX Channels ======
#define CH_FRONT_LEFT   4
#define CH_FRONT_CENTER 5
#define CH_FRONT_RIGHT  6
#define CH_LEFT_SIDE    3
#define CH_RIGHT_SIDE   7

// ====== Encoder Pins ======
#define ENCA1 35
#define ENCA2 34
#define ENCB1 33
#define ENCB2 32

// ====== Motor Pins ======
#define L_IN1 5
#define L_IN2 17
#define L_PWM 18

#define R_IN1 2
#define R_IN2 4
#define R_PWM 15

// ====== PWM Channels ======
#define CH_L 0
#define CH_R 1

// ====== Globals ======
Adafruit_VL6180X tof;
MPU9250_asukiaaa mpu;





// Encoders
volatile int posA = 0;
volatile int posB = 0;
volatile int lastEncodedA = 0;
volatile int lastEncodedB = 0;

// ====== Distance Calibration ======
float TICKS_PER_MM_L = 13.56f;
float TICKS_PER_MM_R = 13.45f;

// ====== IMU ======
float yaw = 0;
float gyroBiasZ = 0;
unsigned long lastYawUpdate = 0;

// ====== ToF calibration ======
struct Cal { float a, b; };
Cal CAL[] = {
  { 1.0667f, -49.3333f }, 
  { 0.9524f,  19.0476f }, 
  { 1.0000f, -36.0000f }, 
  { 0.9756f,  -4.3902f }, 
  { 1.0811f, -29.7297f }
};

enum MazeState {
    STATE_A_BOTH_SIDES,
    STATE_B_LEFT_ONLY,
    STATE_C_RIGHT_ONLY,
    STATE_D_NO_WALLS,
    STATE_E_FRONT,
    STATE_F_FRONT_LEFT,
    STATE_G_FRONT_RIGHT
};





// ====== Encoder ISRs ======
void IRAM_ATTR updateEncoderA() {
  int msb = digitalRead(ENCA1);
  int lsb = digitalRead(ENCA2);
  int encoded = (msb << 1) | lsb;
  int sum = (lastEncodedA << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
    posA++;
  else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
    posA--;

  lastEncodedA = encoded;
}

void IRAM_ATTR updateEncoderB() {
  int msb = digitalRead(ENCB1);
  int lsb = digitalRead(ENCB2);
  int encoded = (msb << 1) | lsb;
  int sum = (lastEncodedB << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
    posB--;
  else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
    posB++;

  lastEncodedB = encoded;
}

// ====== Motor Control ======
void motorsBegin() {
  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);
  pinMode(R_IN1, OUTPUT);
  pinMode(R_IN2, OUTPUT);
  ledcSetup(CH_L, 1000, 8);
  ledcSetup(CH_R, 1000, 8);
  ledcAttachPin(L_PWM, CH_L);
  ledcAttachPin(R_PWM, CH_R);
}

void motorL(int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm > 0) {
    digitalWrite(L_IN1, HIGH);
    digitalWrite(L_IN2, LOW);
    ledcWrite(CH_L, pwm);
  } else if (pwm < 0) {
    digitalWrite(L_IN1, LOW);
    digitalWrite(L_IN2, HIGH);
    ledcWrite(CH_L, -pwm);
  } else {
    digitalWrite(L_IN1, LOW);
    digitalWrite(L_IN2, LOW);
    ledcWrite(CH_L, 0);
  }
}

void motorR(int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm > 0) {
    digitalWrite(R_IN1, HIGH);
    digitalWrite(R_IN2, LOW);
    ledcWrite(CH_R, pwm);
  } else if (pwm < 0) {
    digitalWrite(R_IN1, LOW);
    digitalWrite(R_IN2, HIGH);
    ledcWrite(CH_R, -pwm);
  } else {
    digitalWrite(R_IN1, LOW);
    digitalWrite(R_IN2, LOW);
    ledcWrite(CH_R, 0);
  }
}

void motorsStop() {
  motorL(0);
  motorR(0);
}

float angleDiff(float target, float current) {
  float diff = target - current;
  while (diff > 180) diff -= 360;
  while (diff < -180) diff += 360;
  return diff;
}

// ====== MUX ======
void mux(uint8_t ch) {
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
  delayMicroseconds(200);
}

float readMM(uint8_t ch, int idx) {
  mux(ch);
  delay(2);
  uint8_t r = tof.readRange();
  if (tof.readRangeStatus() != VL6180X_ERROR_NONE) return -1.0f;
  float mm = CAL[idx].a * r + CAL[idx].b;
  if (mm < 0) mm = 0;
  return mm;
}

// ====== IMU ======
void imuBegin() {
  mpu.setWire(&Wire);
  mpu.beginAccel();
  mpu.beginGyro();
  //mpu.beginMag();

  delay(1200);

  double sum = 0;
  for (int i = 0; i < 200; i++) {
    mpu.gyroUpdate();
    sum += mpu.gyroZ();
    delay(5);
  }
  gyroBiasZ = sum / 200.0;

  yaw = 0;
  lastYawUpdate = millis();
}


MazeState detectMazeState() {

    float LS = readMM(CH_LEFT_SIDE, 0);
    float FL = readMM(CH_FRONT_LEFT, 1);
    float FC = readMM(CH_FRONT_CENTER, 2);
    float FR = readMM(CH_FRONT_RIGHT, 3);
    float RS = readMM(CH_RIGHT_SIDE, 4);

    bool leftOK   = (LS >= 0 && LS < 150);
    bool rightOK  = (RS >= 0 && RS < 150);

    bool front = (FL > 0 && FR > 0 && FL < 70 && FR < 70);

    if (front && leftOK && !rightOK) return STATE_F_FRONT_LEFT;
    if (front && rightOK && !leftOK) return STATE_G_FRONT_RIGHT;
    if (front)                      return STATE_E_FRONT;

    if (leftOK && rightOK) return STATE_A_BOTH_SIDES;
    if (leftOK && !rightOK) return STATE_B_LEFT_ONLY;
    if (!leftOK && rightOK) return STATE_C_RIGHT_ONLY;

    return STATE_D_NO_WALLS;
}




void preciseStopCorrection(long targetTicksL, long targetTicksR) {
  // Compute current travelled ticks
  noInterrupts();
  long curL = posA - targetTicksL;
  long curR = posB - targetTicksR;
  interrupts();

  // Convert to mm
  float distL = curL / TICKS_PER_MM_L;
  float distR = curR / TICKS_PER_MM_R;
  float avgErr = (distL + distR) * 0.5f;   // mm error

  // If within ±3mm → perfect
  if (abs(avgErr) <= 3.0f) return;

  int pwm = 160;  // correction power

  if (avgErr > 3) {
    // Overshoot → reverse a little
    motorL(-pwm);
    motorR(-pwm);
    delay(abs(avgErr) * 4); // ~4ms per mm
  } else {
    // Undershoot → move forward a little
    motorL(pwm);
    motorR(pwm);
    delay(abs(avgErr) * 4);
  }

  motorsStop();
}









void driveStraightIMU_smooth(float distance_cm, int maxPWM) {

  float distance_mm = distance_cm * 10.0f;

  long startA, startB;
  noInterrupts();
  startA = posA;
  startB = posB;
  interrupts();

  // ==========================
  // ⭐ Step 1 — Update IMU BEFORE movement
  // ==========================
  mpu.gyroUpdate();
  lastYawUpdate = millis();
  float gz = mpu.gyroZ() - gyroBiasZ;
  yaw += gz * 0.001f;   // tiny update

  // ⭐ NOW lock real starting heading
  float targetYaw = yaw;

  // ==========================
  // Gain & ramp config
  // ==========================
  const float Kp = 4.6f;
  const float rampDist = 30.0f;    
  const int MIN_START_PWM = 70;    
  const int MIN_RUNNING_PWM = 50;  

  // ==========================
  // ⭐ Step 2 — Kickstart AFTER locking heading
  // ==========================
  motorL(MIN_START_PWM);
  motorR(MIN_START_PWM);
  delay(40);  // give wheels initial torque

  // ==========================
  // ⭐ Step 3 — Main IMU loop
  // ==========================
  while (true) {

    // --- IMU update ---
    mpu.gyroUpdate();
    unsigned long now = millis();
    float dt = (now - lastYawUpdate)/1000.0f;
    lastYawUpdate = now;

    gz = mpu.gyroZ() - gyroBiasZ;
    yaw += gz * dt;

    if (yaw > 180) yaw -= 360;
    if (yaw < -180) yaw += 360;

    // --- Encoder distance ---
    noInterrupts();
    long curA = posA - startA;
    long curB = posB - startB;
    interrupts();

    float distL = curA / TICKS_PER_MM_L;
    float distR = curB / TICKS_PER_MM_R;
    float avg = (distL + distR) * 0.5f;

    if (avg >= distance_mm) break;

    // --- PWM ramp ---
    float pwm = maxPWM;

    if (avg < rampDist) {
      pwm = MIN_START_PWM + (maxPWM - MIN_START_PWM) * (avg / rampDist);
    }
    else if ((distance_mm - avg) < rampDist) {
      pwm = MIN_RUNNING_PWM + (maxPWM - MIN_RUNNING_PWM) * ((distance_mm - avg) / rampDist);
    }

    pwm = constrain(pwm, MIN_RUNNING_PWM, maxPWM);

    // --- Heading correction ---
    float err = angleDiff(targetYaw, yaw);
    float turn = Kp * err;

    int pwmL = pwm - turn;
    int pwmR = pwm + turn;

    motorL(pwmL);
    motorR(pwmR);

    delay(5);
  }

  // ==========================
  // ⭐ Step 4 — Hard brake + reverse pulse
  // ==========================


  motorL(-60);
  motorR(-60);
  delay(35);

  motorsStop();

  // After reverse pulse
motorsStop();

// Encoder-based fine correction
long targetA = startA + distance_mm * TICKS_PER_MM_L;
long targetB = startB + distance_mm * TICKS_PER_MM_R;
preciseStopCorrection(targetA, targetB);


}




float angleError_FL_FR() {
    float FL = readMM(CH_FRONT_LEFT, 1);
    float FR = readMM(CH_FRONT_RIGHT, 3);

    // valid range for 45° sensors
    if (FL < 20 || FL > 150) return 0;
    if (FR < 20 || FR > 150) return 0;

    // angle error = difference
    float diff = FL - FR;

    // tuning gain
    const float Kp = 1.4f;

    return Kp * diff;  // positive = turn left, negative = turn right
}




// ===== BUZZER FUNCTIONS (PASSIVE BUZZER) =====

#define BUZZER_CH 3        // LEDC channel for buzzer
#define BUZZER_FREQ 4000   // 4 kHz
#define BUZZER_RES 8       // 8-bit resolution

void buzzInit() {
    ledcSetup(BUZZER_CH, BUZZER_FREQ, BUZZER_RES);
    ledcAttachPin(BUZZER_PIN, BUZZER_CH);
    ledcWrite(BUZZER_CH, 0);   // off
}

void buzzShort() {
    ledcWrite(BUZZER_CH, 200);  // volume
    delay(60);
    ledcWrite(BUZZER_CH, 0);
}

void buzzLong() {
    ledcWrite(BUZZER_CH, 200);
    delay(250);
    ledcWrite(BUZZER_CH, 0);
}

void buzzConfirm(int n = 1) {
    for (int i = 0; i < n; i++) {
        buzzShort();
        delay(80);
    }
}


void driveCentered45(int basePWM) {

    float FL = readMM(CH_FRONT_LEFT, 1);
    float FR = readMM(CH_FRONT_RIGHT, 3);

    // invalid → just go straight
    if (FL < 0 || FR < 0) {
        motorL(basePWM);
        motorR(basePWM);
        return;
    }

    float diff = FL - FR;  // positive → tilted right

    const float Kp = 1.2f;   // mild correction
    float turn = Kp * diff;

    int pwmL = basePWM - turn;
    int pwmR = basePWM + turn;

    motorL(pwmL);
    motorR(pwmR);
}

bool frontWallDetected(float FL, float FR) {
    if (FL < 0 || FR < 0) return false;

    // When front wall exists, both drop to ~60–80mm
    if (FL < 85 && FR < 85) return true;

    return false;
}

void printSensors() {
    float LS = readMM(CH_LEFT_SIDE,    0);
    float FL = readMM(CH_FRONT_LEFT,   1);
    float FC = readMM(CH_FRONT_CENTER, 2);
    float FR = readMM(CH_FRONT_RIGHT,  3);
    float RS = readMM(CH_RIGHT_SIDE,   4);

    Serial.printf("LS:%5.1f  FL:%5.1f  FC:%5.1f  FR:%5.1f  RS:%5.1f    ",
                  LS, FL, FC, FR, RS);
}


void printState(MazeState s) {
    switch (s) {
        case STATE_A_BOTH_SIDES:  Serial.println("STATE A: Both side walls"); break;
        case STATE_B_LEFT_ONLY:   Serial.println("STATE B: Left wall only"); break;
        case STATE_C_RIGHT_ONLY:  Serial.println("STATE C: Right wall only"); break;
        case STATE_D_NO_WALLS:    Serial.println("STATE D: No walls"); break;
        case STATE_E_FRONT:       Serial.println("STATE E: FRONT WALL"); break;
        case STATE_F_FRONT_LEFT:  Serial.println("STATE F: FRONT + LEFT"); break;
        case STATE_G_FRONT_RIGHT: Serial.println("STATE G: FRONT + RIGHT"); break;

        
    }
}


bool frontBeeped = false;

// ====== Setup ======
void setup() {
  
  Serial.begin(115200);
  delay(500);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  motorsBegin();

  // ToF init
  uint8_t channels[5] = {CH_LEFT_SIDE, CH_FRONT_LEFT, CH_FRONT_CENTER, CH_FRONT_RIGHT, CH_RIGHT_SIDE};
  for (int i = 0; i < 5; i++) {
    mux(channels[i]);
    delay(3);
    tof.begin(&Wire);
  }

  imuBegin();

  pinMode(BTN_PIN, INPUT);   // Button gives HIGH when pressed

  // Encoder init
  pinMode(ENCA1, INPUT_PULLUP);
  pinMode(ENCA2, INPUT_PULLUP);
  pinMode(ENCB1, INPUT_PULLUP);
  pinMode(ENCB2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA1), updateEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCA2), updateEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB1), updateEncoderB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB2), updateEncoderB, CHANGE);

  buzzInit();
  buzzConfirm(1);

 

  buzzShort();
}

// ====== Loop (run straight once) ======
bool runOnce = false;



void driveSideWalls(int basePWM)
{
    float L = readMM(CH_LEFT_SIDE, 0);
    float R = readMM(CH_RIGHT_SIDE, 4);

    bool leftOK  = (L > 10 && L < 180);
    bool rightOK = (R > 10 && R < 180);

    float turn = 0;

    // ===== CASE 1: BOTH WALLS → CENTER =====
    if (leftOK && rightOK) {
        float diff = (R - L);   // positive: robot too close to left
        const float Kp = 0.9f;  // mild

        turn = Kp * diff;
    }

    // ===== CASE 2: LEFT ONLY =====
    else if (leftOK && !rightOK) {
        float err = (L - 40.0f); // target distance = 40 mm
        const float Kp = 1.1f;

        turn = Kp * err;   // positive → drift to right
    }

    // ===== CASE 3: RIGHT ONLY =====
    else if (!leftOK && rightOK) {
        float err = (R - 40.0f);
        const float Kp = 1.1f;

        turn = -Kp * err;  // negative → drift to left
    }

    // ===== CASE 4: NO WALLS → GO STRAIGHT =====
    else {
        motorL(basePWM);
        motorR(basePWM);
        return;
    }

    // ===== APPLY MOTOR CORRECTION =====
    int pwmL = basePWM - turn;
    int pwmR = basePWM + turn;

    motorL(pwmL);
    motorR(pwmR);
}









void driveStraight_IMU_untilFrontWall(int basePWM)
{
    // ---- Lock starting heading ----
    mpu.gyroUpdate();
    lastYawUpdate = millis();
    float targetYaw = yaw;

    const float KpYaw = 4.2f;    // IMU correction
    const int MIN_PWM = 60;

    while (true)
    {
        // ----- IMU update -----
        mpu.gyroUpdate();
        unsigned long now = millis();
        float dt = (now - lastYawUpdate) / 1000.0f;
        lastYawUpdate = now;

        float gz = mpu.gyroZ() - gyroBiasZ;
        yaw += gz * dt;

        if (yaw > 180) yaw -= 360;
        if (yaw < -180) yaw += 360;

        // ----- Check front wall -----
        float FL = readMM(CH_FRONT_LEFT, 1);
        float FR = readMM(CH_FRONT_RIGHT, 3);


        bool front = (FL > 0 && FR > 0 && FL < 70 && FR < 70);
        if (front) {
            motorsStop();
            buzzConfirm(2);
            Serial.println("=== FRONT WALL STOP ===");
            return;
        }

        // ----- IMU correction only -----
        float yawErr = angleDiff(targetYaw, yaw);
        float turn = KpYaw * yawErr;

        int pwmL = basePWM - turn;
        int pwmR = basePWM + turn;

        // safety lower bound
        if (abs(pwmL) < MIN_PWM) pwmL = MIN_PWM * (pwmL >= 0 ? 1 : -1);
        if (abs(pwmR) < MIN_PWM) pwmR = MIN_PWM * (pwmR >= 0 ? 1 : -1);

        motorL(pwmL);
        motorR(pwmR);

        // Debug
        Serial.printf("FL=%.1f FR=%.1f yaw=%.2f err=%.2f pwmL=%d pwmR=%d\n",
                       FL, FR, yaw, yawErr, pwmL, pwmR);

        delay(10);
    }
}


void squareUpFront() {
          buzzConfirm(1);

    while (true) {
        float FL = readMM(CH_FRONT_LEFT, 1);
        float FR = readMM(CH_FRONT_RIGHT, 3);

        // both sensors must see the wall closely
        if (FL < 45 && FR < 45 && FL > 0 && FR > 0) {
            motorsStop();
            delay(100);

            // reverse a tiny bit for turn clearance
            motorL(-80);
            motorR(-80);
            delay(80);
            motorsStop();

            // re-zero IMU yaw
            yaw = 0;
            lastYawUpdate = millis();

            Serial.println("=== SQUARED UP ===");
            return;
        }

        // move slowly forward
        motorL(90);
        motorR(90);
        delay(10);
    }
}

void turnLeft90() {
    float target = yaw + 90;
    if (target < -180) target += 360;

    while (fabs(angleDiff(target, yaw)) > 2) {
        mpu.gyroUpdate();
        unsigned long now = millis();
        float dt = (now - lastYawUpdate)/1000.0f;
        lastYawUpdate = now;
        yaw += (mpu.gyroZ() - gyroBiasZ) * dt;

        motorL(-120);
        motorR(120);
    }

    motorsStop();
    buzzConfirm(1);
}

void turnRight90() {
    float target = yaw - 90;
    if (target > 180) target -= 360;

    while (fabs(angleDiff(target, yaw)) > 2) {
        mpu.gyroUpdate();
        unsigned long now = millis();
        float dt = (now - lastYawUpdate)/1000.0f;
        lastYawUpdate = now;
        yaw += (mpu.gyroZ() - gyroBiasZ) * dt;

        motorL(120);
        motorR(-120);
    }

    motorsStop();
    buzzConfirm(1);
}

void beepWhilePressed() {
    if (digitalRead(BTN_PIN) == HIGH) {
        ledcWrite(BUZZER_CH, 200); // beep ON
    } else {
        ledcWrite(BUZZER_CH, 0);   // beep OFF
    }
}


void loop() {

    MazeState state = detectMazeState();
    printSensors();
    printState(state);

    // Reset latch when moved away from front states
    if (state != STATE_E_FRONT &&
        state != STATE_F_FRONT_LEFT &&
        state != STATE_G_FRONT_RIGHT) {
        frontBeeped = false;
    }

    switch (state) {

    case STATE_A_BOTH_SIDES:
        driveStraight_IMU_untilFrontWall(180);
        squareUpFront();
        turnRight90();

        break;

    case STATE_B_LEFT_ONLY:
        driveSideWalls(120);
        break;

    case STATE_C_RIGHT_ONLY:
        driveSideWalls(120);
        break;

    case STATE_D_NO_WALLS:
        motorL(140);
        motorR(140);
        break; 

    case STATE_E_FRONT:
        motorsStop();
        if (!frontBeeped) { buzzConfirm(2); frontBeeped = true; }
        break;

    case STATE_F_FRONT_LEFT:
        motorsStop();
        if (!frontBeeped) { buzzConfirm(3); frontBeeped = true; }
        break;

    case STATE_G_FRONT_RIGHT:
        motorsStop();
        if (!frontBeeped) { buzzConfirm(3); frontBeeped = true; }
        break;
}


    delay(30);
}


// void loop() {
//     beepWhilePressed();
// }
