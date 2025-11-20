#include <Wire.h>
#include <Arduino.h>
#include "Adafruit_VL6180X.h"
#include <MPU9250_asukiaaa.h>

// ====== I²C and MUX Pins ======
#define SDA_PIN 26
#define SCL_PIN 25
#define MUX_ADDR 0x70

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
  {1.00f, -42.0f},
  {1.00f,  20.0f},
  {1.00f, -35.0f},
  {1.00f, -10.0f},
  {1.00f, -28.0f}
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
  delay(65);

  motorsStop();
}







// ====== Setup ======
void setup() {
  Serial.begin(115200);
  delay(500);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  motorsBegin();

  // ToF init
  uint8_t channels[5] = {CH_FRONT_LEFT, CH_FRONT_CENTER, CH_FRONT_RIGHT, CH_LEFT_SIDE, CH_RIGHT_SIDE};
  for (int i = 0; i < 5; i++) {
    mux(channels[i]);
    delay(3);
    tof.begin(&Wire);
  }

  imuBegin();

  // Encoder init
  pinMode(ENCA1, INPUT_PULLUP);
  pinMode(ENCA2, INPUT_PULLUP);
  pinMode(ENCB1, INPUT_PULLUP);
  pinMode(ENCB2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA1), updateEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCA2), updateEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB1), updateEncoderB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB2), updateEncoderB, CHANGE);
}

// ====== Loop (run straight once) ======
bool runOnce = false;

void loop() {
  if (!runOnce) {
    delay(1000);
    driveStraightIMU_smooth(50.0f, 200);   // drive 100 cm at pwm=120
    runOnce = true;
  }
}
