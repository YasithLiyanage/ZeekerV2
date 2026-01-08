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


// --- Corridor Center Calibration ---
const float CENTER_OFFSET_MM = -3.4f;   // true L-R diff when perfectly centered



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
    // ---- Reset yaw reference ----
    yaw = 0;
    mpu.gyroUpdate();
    lastYawUpdate = millis();
    float targetYaw = 0;   // Always correct reference

    const float KpYaw = 4.2f;    
    const int MIN_PWM = 60;

    while (true)
    {
        // ----- IMU update -----
        mpu.gyroUpdate();
        unsigned long now = millis();
        float dt = (now - lastYawUpdate) / 1000.0f;
        lastYawUpdate = now;

        float gz = mpu.gyroZ() - gyroBiasZ;   // bias removed
        yaw += gz * dt;

        if (yaw > 180) yaw -= 360;
        if (yaw < -180) yaw += 360;

        // ----- FRONT WALL DETECTION -----
        float FL = readMM(CH_FRONT_LEFT, 1);
        float FR = readMM(CH_FRONT_RIGHT, 3);

        bool front = (FL > 0 && FR > 0 && FL < 70 && FR < 70);

        if (front) {
            motorsStop();
            buzzConfirm(2);
            Serial.println("=== FRONT WALL STOP ===");
            return;
        }

        // ----- IMU heading correction -----
        float yawErr = angleDiff(targetYaw, yaw);
        float turn = KpYaw * yawErr;

        int pwmL = basePWM - turn;
        int pwmR = basePWM + turn;

        if (abs(pwmL) < MIN_PWM) pwmL = MIN_PWM * (pwmL >= 0 ? 1 : -1);
        if (abs(pwmR) < MIN_PWM) pwmR = MIN_PWM * (pwmR >= 0 ? 1 : -1);

        motorL(pwmL);
        motorR(pwmR);

        Serial.printf("FL=%.1f FR=%.1f | yaw=%.2f err=%.2f | L=%d R=%d\n",
                      FL, FR, yaw, yawErr, pwmL, pwmR);

        delay(10);
    }
}


void driveForwardSmart(int basePWM)
{
    // ===== Constants =====
    const float SIDE_TARGET_MM = 40.0f;
    const float Kp_bothWalls   = 0.9f;
    const float Kp_singleWall  = 1.1f;
    const float Kp_yaw         = 4.2f;
    const int   MIN_PWM        = 55;

    // ===== Sensor Read =====
    float L  = readMM(CH_LEFT_SIDE,  0);
    float R  = readMM(CH_RIGHT_SIDE, 4);
    float FL = readMM(CH_FRONT_LEFT, 1);
    float FR = readMM(CH_FRONT_RIGHT,3);

    bool leftOK  = (L > 10 && L < 180);
    bool rightOK = (R > 10 && R < 180);

    // ===== FRONT WALL DETECTION (45° SENSORS) =====
    bool frontWall = (FL > 0 && FR > 0 && FL < 70 && FR < 70);
    if (frontWall) {
        motorsStop();
        buzzConfirm(2);
        Serial.println("=== FRONT WALL DETECTED ===");
        return;
    }

    // ===== IMU UPDATE (always keep yaw fresh) =====
    mpu.gyroUpdate();
    unsigned long now = millis();
    float dt = (now - lastYawUpdate) / 1000.0f;
    lastYawUpdate = now;

    float gz = mpu.gyroZ() - gyroBiasZ;
    yaw += gz * dt;

    if (yaw > 180) yaw -= 360;
    if (yaw < -180) yaw += 360;

    static float targetYaw = yaw;   // auto-locks when no walls

    float turn = 0;

    // ==================================================
    // ===== CASE 1: BOTH SIDE WALLS → CENTER =====
    // ==================================================
    if (leftOK && rightOK) {
        float diff = (L - R);      // +ve = too close left
        turn = Kp_bothWalls * diff;

        // reset IMU target when walls are reliable
        targetYaw = yaw;
    }

    // ==================================================
    // ===== CASE 2: LEFT WALL ONLY =====
    // ==================================================
    else if (leftOK && !rightOK) {
        float err = (L - SIDE_TARGET_MM);
        turn = Kp_singleWall * err;
        targetYaw = yaw;
    }

    // ==================================================
    // ===== CASE 3: RIGHT WALL ONLY =====
    // ==================================================
    else if (!leftOK && rightOK) {
        float err = (R - SIDE_TARGET_MM);
        turn = -Kp_singleWall * err;
        targetYaw = yaw;
    }

    // ==================================================
    // ===== CASE 4: NO WALLS → IMU STRAIGHT =====
    // ==================================================
    else {
        float yawErr = angleDiff(targetYaw, yaw);
        turn = Kp_yaw * yawErr;
    }

    // ===== APPLY MOTOR OUTPUT =====
    int pwmL = basePWM - turn;
    int pwmR = basePWM + turn;

    // safety floor
    if (abs(pwmL) < MIN_PWM) pwmL = MIN_PWM * (pwmL >= 0 ? 1 : -1);
    if (abs(pwmR) < MIN_PWM) pwmR = MIN_PWM * (pwmR >= 0 ? 1 : -1);

    motorL(pwmL);
    motorR(pwmR);

    // ===== DEBUG (optional) =====
    Serial.printf("L:%5.1f R:%5.1f FL:%5.1f FR:%5.1f yaw:%6.2f pwmL:%d pwmR:%d\n",
                  L, R, FL, FR, yaw, pwmL, pwmR);
}



void turnIMU(float angleDeg) {
    // angleDeg: +90 = left, -90 = right

    // Lock start time
    mpu.gyroUpdate();
    lastYawUpdate = millis();

    float startYaw = yaw;
    float targetYaw = startYaw + angleDeg;

    if (targetYaw > 180)  targetYaw -= 360;
    if (targetYaw < -180) targetYaw += 360;

    const float Kp = 3.6f;     // position gain
    const float Kd = 0.35f;    // damping
    const int   MAX_PWM = 140;
    const int   MIN_PWM = 55;

    float prevErr = angleDiff(targetYaw, yaw);

    while (true) {
        // ----- IMU update -----
        mpu.gyroUpdate();
        unsigned long now = millis();
        float dt = (now - lastYawUpdate) / 1000.0f;
        lastYawUpdate = now;

        float gz = mpu.gyroZ() - gyroBiasZ;
        yaw += gz * dt;

        if (yaw > 180) yaw -= 360;
        if (yaw < -180) yaw += 360;

        // ----- Angle error -----
        float err = angleDiff(targetYaw, yaw);

        // Exit window
        if (fabs(err) < 2.0f) break;

        // ----- PD control -----
        float derr = (err - prevErr) / dt;
        prevErr = err;

        float control = (Kp * err) + (Kd * derr);

        // ----- PWM shaping -----
        int pwm = constrain(abs(control), MIN_PWM, MAX_PWM);

        // Slow down near target
        if (fabs(err) < 25) {
            pwm = map(fabs(err), 0, 25, MIN_PWM, pwm);
        }

        if (control > 0) {
            motorL(-pwm);
            motorR( pwm);
        } else {
            motorL( pwm);
            motorR(-pwm);
        }

        delay(4);
    }

    // ----- Active brake -----
    motorL(60);
    motorR(-60);
    delay(25);

    motorsStop();
    delay(40);

    // Snap yaw exactly
    yaw = targetYaw;

    buzzConfirm(1);
}


void turnLeft90() {

  turnIMU(+90.0f);

}

void turnRight90() {
  
  turnIMU(-90.0f);

}

void turn180() {
  
  turnIMU(+180.0f);

}

void beepWhilePressed() {
    if (digitalRead(BTN_PIN) == HIGH) {
        ledcWrite(BUZZER_CH, 200); // beep ON
    } else {
        ledcWrite(BUZZER_CH, 0);   // beep OFF
    }
}


// ========================================
// PROFESSIONAL DRIVE STRAIGHT FUNCTION - FIXED
// ========================================

struct DriveState {
  float targetYaw;
  bool yawLocked;
  unsigned long lastUpdate;
  float integralError;  // NEW: for PI control
  unsigned long lastWallTime; // NEW: track when we last saw walls
};

DriveState driveState = {0, false, 0, 0, 0};

void initDriveStraight() {
  // Initialize driving state
  mpu.gyroUpdate();
  driveState.targetYaw = yaw;
  driveState.yawLocked = true;
  driveState.lastUpdate = millis();
  driveState.integralError = 0;  // Reset integral
  driveState.lastWallTime = millis();
}


void driveReliable(int basePWM) {

    // ===== Config =====
    const float TARGET = 40.0f;
    const float MIN_WALL = 15.0f;
    const float MAX_WALL = 120.0f;

    const float Kp_wall   = 1.05f;
    const float Kp_center = 0.55f;     // reduced
    const float Kp_yaw    = 3.5f;

    const float CENTER_OFFSET = -2.0f; // based on your measurements
    const float DIFF_DEADBAND = 2.0f;  // ignore noise

    const int MIN_PWM = 50;

    // ===== Read sensors =====
    float L  = readMM(CH_LEFT_SIDE, 0);
    float R  = readMM(CH_RIGHT_SIDE, 4);
    float FL = readMM(CH_FRONT_LEFT, 1);
    float FR = readMM(CH_FRONT_RIGHT, 3);
    float FC = readMM(CH_FRONT_CENTER, 2);

    bool leftOK  = (L > MIN_WALL && L < MAX_WALL);
    bool rightOK = (R > MIN_WALL && R < MAX_WALL);

    // ===== Update IMU =====
    mpu.gyroUpdate();
    unsigned long now = millis();
    float dt = (now - lastYawUpdate) / 1000.0f;
    lastYawUpdate = now;

    float gz = mpu.gyroZ() - gyroBiasZ;
    yaw += gz * dt;
    if (yaw > 180) yaw -= 360;
    if (yaw < -180) yaw += 360;

    static float targetYaw = yaw;

    float turn = 0;

    // ===== PRINT ALL RAW SENSOR VALUES =====
    Serial.printf("LS:%5.1f FL:%5.1f FC:%5.1f FR:%5.1f RS:%5.1f | yaw:%6.2f | ",
                  L, FL, FC, FR, R, yaw);

    // ===========================================================
    // CASE 1 — BOTH WALLS → center
    // ===========================================================
    if (leftOK && rightOK) {

        float diff = (L - R) - CENTER_OFFSET;

        // deadband for noise
        if (fabs(diff) < DIFF_DEADBAND) diff = 0;

        turn = Kp_center * diff;
        targetYaw = yaw;

        Serial.printf("[BOTH] diff=%.2f turn=%.2f\n", diff, turn);
    }

    // ===========================================================
    // CASE 2 — LEFT WALL ONLY
    // ===========================================================
    else if (leftOK && !rightOK) {

        float err = L - TARGET;
        turn = Kp_wall * err;
        targetYaw = yaw;

        Serial.printf("[LEFT] err=%.1f turn=%.1f\n", err, turn);
    }

    // ===========================================================
    // CASE 3 — RIGHT WALL ONLY
    // ===========================================================
    else if (!leftOK && rightOK) {

        float err = R - TARGET;
        turn = -Kp_wall * err;

        targetYaw = yaw;

        Serial.printf("[RIGHT] err=%.1f turn=%.1f\n", err, turn);
    }

    // ===========================================================
    // CASE 4 — NO WALLS → IMU
    // ===========================================================
    else {
        float yawErr = angleDiff(targetYaw, yaw);
        turn = Kp_yaw * yawErr;

        Serial.printf("[IMU] yawErr=%.2f turn=%.2f\n", yawErr, turn);
    }

    // ===== Apply motor output =====
    int pwmL = basePWM - turn;
    int pwmR = basePWM + turn;

    if (abs(pwmL) < MIN_PWM) pwmL = MIN_PWM * (pwmL >= 0 ? 1 : -1);
    if (abs(pwmR) < MIN_PWM) pwmR = MIN_PWM * (pwmR >= 0 ? 1 : -1);

    motorL(pwmL);
    motorR(pwmR);
}



// void loop() {
//   // Initialize drive state once at start
//   static bool initialized = false;
//   if (!initialized) {
//     initDriveStraight();
//     initialized = true;
//   }
  
//   MazeState state = detectMazeState();
//   printSensors();
//   printState(state);
  
//   switch (state) {
//     case STATE_A_BOTH_SIDES:
//     case STATE_B_LEFT_ONLY:
//     case STATE_C_RIGHT_ONLY:
//     case STATE_D_NO_WALLS:
//       driveStraightPro(160);  // Drive at 160 base PWM
//       break;
      
//     case STATE_E_FRONT:
//     case STATE_F_FRONT_LEFT:
//     case STATE_G_FRONT_RIGHT:
//       motorsStop();
//       squareUpFront();
      
//       // Decide turn direction based on available paths
//       if (state == STATE_F_FRONT_LEFT) {
//         turnRight90();
//       } else if (state == STATE_G_FRONT_RIGHT) {
//         turnLeft90();
//       } else {
//         turnRight90();  // Default turn
//       }
      
//       // Re-initialize after turn
//       initDriveStraight();
//       break;
//   }
  
//   delay(10);  // Reduced delay for faster control loop
// }
// ===============================
//  Corridor Centering with PID
// ===============================

float prevDiff = 0;
float integral  = 0;

void driveCenteredCorridor(int basePWM)
{
    float L = readMM(CH_LEFT_SIDE, 0);
    float R = readMM(CH_RIGHT_SIDE, 4);

    bool leftOK  = (L > 15 && L < 120);
    bool rightOK = (R > 15 && R < 120);

    if (!leftOK && !rightOK) {
        integral = 0;            // prevent wind-up
        motorL(basePWM);
        motorR(basePWM);
        return;
    }

    const float CENTER_OFFSET = 0.0f;
    const float DEADBAND = 2.0f;

    float rawDiff = L - R;
    float diff = rawDiff - CENTER_OFFSET;

    if (fabs(diff) < DEADBAND) diff = 0;

    // ===== PID Gains =====
    const float Kp = 2.35f;
    const float Kd = 1.35f; //0.28
    const float Ki = 0.03f;   //0.03   // **EXTREMELY small**

    // ===== Integral logic =====
    if (fabs(diff) < 8.0f) {     // only integrate when near center
        integral += diff;
        // clamp to prevent runaway
        if (integral > 40) integral = 40;
        if (integral < -40) integral = -40;
    } else {
        integral = 0;            // reset when far → prevents overshoot
    }

    // ===== Derivative =====
    float derivative = diff - prevDiff;
    prevDiff = diff;

    // ===== PID Output =====
    float turn = (Kp * diff) + (Kd * derivative) + (Ki * integral);

    // ===== Motor Output =====
    int pwmL = basePWM - turn;
    int pwmR = basePWM + turn;

    motorL(pwmL);
    motorR(pwmR);

    Serial.printf(
        "L=%.1f R=%.1f | diff=%.2f d=%.2f I=%.1f | turn=%.2f | L=%d R=%d\n",
        L, R, diff, derivative, integral, turn, pwmL, pwmR
    );
}// ===============================
//  STATE MACHINE TYPES
// ===============================
enum MouseState {
    DRIVE_CENTERED,
    FOLLOW_LEFT,
    FOLLOW_RIGHT,
    DRIVE_IMU,
    FRONT_STOP
};


// ===============================
//  CHOOSE STATE BASED ON WALLS
// ===============================
MouseState chooseState(float L, float R, float FL, float FR) {

    bool leftOK  = (L > 15 && L < 120);
    bool rightOK = (R > 15 && R < 120);

    bool frontOK = (FL < 70 && FR < 70); // front wall detected

    Serial.printf(
        "STATE CHECK | L=%.1f R=%.1f FL=%.1f FR=%.1f | leftOK=%d rightOK=%d frontOK=%d\n",
        L, R, FL, FR, leftOK, rightOK, frontOK
    );

    if (frontOK) {
        Serial.println("→ STATE: FRONT_STOP");
        return FRONT_STOP;
    }

    if (leftOK && rightOK) {
        Serial.println("→ STATE: DRIVE_CENTERED");
        return DRIVE_CENTERED;
    }

    if (leftOK && !rightOK) {
        Serial.println("→ STATE: FOLLOW_LEFT");
        return FOLLOW_LEFT;
    }

    if (!leftOK && rightOK) {
        Serial.println("→ STATE: FOLLOW_RIGHT");
        return FOLLOW_RIGHT;
    }

    Serial.println("→ STATE: DRIVE_IMU");
    return DRIVE_IMU;
}



// ===============================
//  FOLLOW LEFT WALL (PD)
// ===============================
// void followLeftWall(int basePWM) {

//     float L = readMM(CH_LEFT_SIDE, 0);
//     const float TARGET = 45.0f;

//     float err = L - TARGET;
//     const float Kp = 1.0f;

//     float turn = Kp * err;
//     int pwmL = basePWM - turn;
//     int pwmR = basePWM + turn;

//     motorL(pwmL);
//     motorR(pwmR);

//     Serial.printf("[FOLLOW_LEFT] L=%.1f err=%.1f turn=%.1f pwmL=%d pwmR=%d\n",
//                   L, err, turn, pwmL, pwmR);
// }



void followLeftWall_PD(int basePWM)
{
    float L = readMM(CH_LEFT_SIDE, 0);

    const float TARGET = 42.0f;
    float err = TARGET - L;   // positive error = too far → move left

    const float Kp = 1.85f;
    const float Kd = 0.55f;

    static float prevErr = 0;
    float d = err - prevErr;
    prevErr = err;

    float turn = Kp * err + Kd * d;

    int pwmL = basePWM + turn;
    int pwmR = basePWM - turn;

    motorL(pwmL);
    motorR(pwmR);

    Serial.printf("[LEFT] L=%.1f err=%.2f d=%.2f turn=%.2f | L=%d R=%d\n",
                   L, err, d, turn, pwmL, pwmR);
}



// ===============================
//  FOLLOW RIGHT WALL (PD)
// ===============================
void followRightWall_PD(int basePWM) {

    float R = readMM(CH_RIGHT_SIDE, 4);

    const float TARGET = 42.0f;
    float err = R - TARGET;

   const float Kp = 1.85f;
    const float Kd = 0.55f;

    static float prevErr = 0;
    float d = err - prevErr;
    prevErr = err;

    float turn = Kp * err + Kd * d;

    int pwmL = basePWM + turn;
    int pwmR = basePWM - turn;

    motorL(pwmL);
    motorR(pwmR);

    Serial.printf("[FOLLOW_RIGHT] R=%.1f err=%.1f turn=%.1f pwmL=%d pwmR=%d\n",
                  R, err, turn, pwmL, pwmR);
}


// ======================================================
//                  STATE MACHINE
// ======================================================
enum State { DRIVING, DECISION };
State robotState = DRIVING;

void squareUpFrontWall()
{
    Serial.println("=== FRONT WALL ALIGN START ===");

    const float STOP_DIST = 25.0f;   // desired stop point
    const float APPROACH_START = 100.0f;
    const float MIN_SAFE = 10.0f;    // FC < 10 → emergency
    const int PWM_MIN = 55;
    const int PWM_MAX = 150;

    int pwm = PWM_MAX;

    // ============================================================
    //  STEP 1 — APPROACH USING ONLY FRONT-CENTER (FC)
    // ============================================================
    while (true)
    {
        float FC = readMM(CH_FRONT_CENTER, 2);
        Serial.printf("[APPROACH] FC=%.1f pwm=%d\n", FC, pwm);

        // Emergency too close
        if (FC > 0 && FC < MIN_SAFE) {
            motorsStop();
            Serial.println("!!! EMERGENCY STOP (too close) !!!");
            break;
        }

        // Normal stop at correct distance
        if (FC > 0 && fabs(FC - STOP_DIST) <= 1.5f) {
            motorsStop();
            break;
        }

        // Invalid (0 / -1): freeze until stable
        if (FC <= 0) {
            motorsStop();
            delay(20);
            continue;
        }

        // Forward
        if (FC > STOP_DIST)
        {
            if (FC < APPROACH_START) {
                pwm = pwm * 0.7;
                if (pwm < PWM_MIN) pwm = PWM_MIN;
            } else {
                pwm = PWM_MAX;
            }
            motorL(pwm);
            motorR(pwm);
        }

        // Backward
        else if (FC < STOP_DIST)
        {
            pwm = pwm * 0.9;
            if (pwm < PWM_MIN) pwm = PWM_MIN;
            motorL(-pwm);
            motorR(-pwm);
        }

        delay(20);
    }

    motorsStop();
    delay(80);


    // ============================================================
    //  STEP 2 — ANGLE ALIGNMENT USING FL & FR ONLY
    // ============================================================
    Serial.println("=== ANGLE CORRECTION START ===");

    const float ANGLE_TOL = 3.0f;  // mm allowed difference

    for (int i = 0; i < 25; i++)
    {
        float FL = readMM(CH_FRONT_LEFT, 1);
        float FR = readMM(CH_FRONT_RIGHT, 3);
        float diff = FL - FR;

        Serial.printf("[ANGLE] FL=%.1f  FR=%.1f  diff=%.2f\n", FL, FR, diff);

        if (fabs(diff) <= ANGLE_TOL) break;

        int turnPWM = (diff > 0) ? -50 : 50;  // rotate toward equality

        motorL(-turnPWM);
        motorR(turnPWM);

        delay(25);
    }

    motorsStop();
    delay(80);

    buzzShort();
    Serial.println("=== FRONT WALL ALIGN DONE ===");

    robotState = DECISION;
}












void driveCentered_PD(int basePWM)
{
    float L = readMM(CH_LEFT_SIDE,0);
    float R = readMM(CH_RIGHT_SIDE,4);

    float diff = L - R;

    if (fabs(diff) < 2.0f) diff = 0;   // deadband

    const float Kp = 0.55f;
    const float Kd = 0.28f;

    float d = diff - prevDiff;
    prevDiff = diff;

    float turn = Kp*diff + Kd*d;

    motorL(basePWM - turn);
    motorR(basePWM + turn);

    Serial.printf("[CENTER] L=%.1f R=%.1f diff=%.2f d=%.2f turn=%.2f\n",
                    L,R,diff,d,turn);
}






// ======================================================
//           MAZE DECISION LOGIC (L F R)
// ======================================================
// void makeMazeDecision()
// {
//     float L  = readMM(CH_LEFT_SIDE, 0);
//     float R  = readMM(CH_RIGHT_SIDE, 4);
//     float FC  = readMM(CH_RIGHT_SIDE, 2);
//     float FL = readMM(CH_FRONT_LEFT, 1);
//     float FR = readMM(CH_FRONT_RIGHT, 3);

//     bool left  = (L  > 15 && L  < 120);
//     bool right = (R  > 15 && R  < 120);
//     bool front = (FL < 70 && FR < 70 && FL > 0 && FR > 0);

//     Serial.printf("[DECISION] L=%d F=%d R=%d → ", left, front, right);
//     Serial.printf("[SENSORS] L=%.1f  FL=%.1f  FC=%.1f  FR=%.1f  R=%.1f\n",
//               L, FL, FC, FR, R);


//     // Basic micromouse rule:
//     if (!left) {
//         Serial.println("TURN LEFT");
//         turnLeft90();
//     }
//     else if (!front) {
//         Serial.println("STRAIGHT");
//         //moveOneCell();          // 18 cm or whatever your cell size
//     }
//     else if (!right) {
//         Serial.println("TURN RIGHT");
//         turnRight90();
//     }
//     else {
//         Serial.println("U-TURN");
//         turn180();
//     }

//     // After completing turn → return to drive mode
//     robotState = DRIVING;
// }

void makeMazeDecision(float L, float R, float FL, float FC, float FR)
{
    Serial.printf("[SENSORS] L=%.1f  FL=%.1f  FC=%.1f  FR=%.1f  R=%.1f\n",
                    L, FL, FC, FR, R);

    bool left  = (L > 15 && L < 90);
    bool right = (R > 15 && R < 90);
    bool front = (FL < 72 && FR < 72 && FL > 0 && FR > 0);

    Serial.printf("[DECISION] L=%d F=%d R=%d → ", left, front, right);

    if (!left) {
        Serial.println("LEFT");
        turnLeft90();
    }
    else if (!front) {
        Serial.println("STRAIGHT");
        //moveForwardOneCell();
    }
    else if (!right) {
        Serial.println("RIGHT");
        turnRight90();
    }
    else {
        Serial.println("U-TURN");
        turn180();
    }

    robotState = DRIVING;
}








// These prevent repeated align loops
bool hasAlignedFront = false;
unsigned long lastAlignTime = 0;
// ======================================================
//                  STATE MACHINE
// ======================================================
//enum State { DRIVING, DECISION };
//State robotState = DRIVING;

const char* stateName(State s) {
    switch (s) {
        case DRIVING:  return "DRIVING";
        case DECISION: return "DECISION";
    }
    return "UNKNOWN";
}


// ======================================================
//          MAIN DRIVE CONTROLLER (CALLED IN LOOP)
// ======================================================
void drive(int basePWM)
{
    // ======= READ ALL SENSORS =======
    float L  = readMM(CH_LEFT_SIDE, 0);
    float FL = readMM(CH_FRONT_LEFT, 1);
    float FC = readMM(CH_FRONT_CENTER, 2);   // optional center ToF
    float FR = readMM(CH_FRONT_RIGHT, 3);
    float R  = readMM(CH_RIGHT_SIDE, 4);

    // ======= DEBUG HEADER OUTPUT =======
    Serial.printf("L=%.1f FL=%.1f FC=%.1f FR=%.1f R=%.1f  |  ",
        L, FL, FC, FR, R
    );

    // ======= FRONT WALL CHECK =======
    bool front = (FL > 0 && FR > 0 && FL < 75 && FR < 75);

    // ======= FRONT ALIGN (once) =======
    if (front && !hasAlignedFront) {

        Serial.println("[STATE=FRONT_ALIGN]");

        motorsStop();
        delay(300);
        buzzConfirm(2);

        squareUpFrontWall();      // this sets robotState = DECISION

        hasAlignedFront = true;
        lastAlignTime = millis();
        return;
    }

    // ======= ALIGN RESET AFTER MOVING =======
    if (hasAlignedFront && millis() - lastAlignTime > 350) {
        hasAlignedFront = false;
    }

    // ======= DECISION MODE =======
    if (robotState == DECISION) {
        Serial.println("[STATE=DECISION]");
        makeMazeDecision(L, R, FL, FC, FR);
        return;
    }

    // ======= SIDE-WALL DETECTION =======
    bool leftOK  = (L > 10 && L < 90);
    bool rightOK = (R > 10 && R < 90);

    // ======= BOTH WALLS (CENTER DRIVE) =======
    if (leftOK && rightOK) {
        Serial.print("[STATE=CENTER] ");
        driveCentered_PD(basePWM);
        return;
    }

    // ======= ONLY LEFT WALL =======
    if (leftOK && !rightOK) {
        Serial.print("[STATE=FOLLOW_LEFT] ");
        followLeftWall_PD(basePWM);
        return;
    }

    // ======= ONLY RIGHT WALL =======
    if (!leftOK && rightOK) {
        Serial.print("[STATE=FOLLOW_RIGHT] ");
        followRightWall_PD(basePWM);
        return;
    }

    // ======= NO WALLS → IMU STRAIGHT =======
    Serial.print("[STATE=IMU] ");
    driveStraight_IMU_untilFrontWall(basePWM);
}







bool runCentered = false;

void loop() {

    // --- Start when button is pressed ---
    if (digitalRead(14) == HIGH && !runCentered) {
        buzzShort();
        delay(200);
        runCentered = true;    // ENTER DRIVE MODE
        prevDiff = 0;          // reset PD
        Serial.println("=== CENTER DRIVE MODE STARTED ===");
        delay(300);
    }

    // --- If running, execute driveCentered every 50ms ---
    if (runCentered) {
        drive(110);
        delay(50);
        return;       // skip idle message
    }

    // --- Idle state ---
    Serial.println("Waiting for button on pin 14...");
    delay(300);
}





