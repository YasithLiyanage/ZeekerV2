#include <Wire.h>
#include <Arduino.h>
#include <MPU9250_asukiaaa.h>

// ====== Motor Pins ======
#define L_IN1 5
#define L_IN2 17
#define L_PWM 18
#define R_IN1 2
#define R_IN2 4
#define R_PWM 15
#define CH_L 0
#define CH_R 1

// ====== IMU Pins ======
#define SDA_PIN 26
#define SCL_PIN 25

MPU9250_asukiaaa mpu;
float yaw = 0;
float gyroBiasZ = 0;
unsigned long lastYawUpdate = 0;

// ====== Calibration Scale (Measured→Real) ======
#define YAW_SCALE 1.006f    // adjust if overshoot/undershoot

// ====== Motors ======
void motorsBegin() {
  pinMode(L_IN1, OUTPUT); pinMode(L_IN2, OUTPUT);
  pinMode(R_IN1, OUTPUT); pinMode(R_IN2, OUTPUT);
  ledcSetup(CH_L, 1000, 8);
  ledcSetup(CH_R, 1000, 8);
  ledcAttachPin(L_PWM, CH_L);
  ledcAttachPin(R_PWM, CH_R);
}

void motorL(int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm > 0) { digitalWrite(L_IN1, HIGH); digitalWrite(L_IN2, LOW); ledcWrite(CH_L, pwm); }
  else if (pwm < 0) { digitalWrite(L_IN1, LOW); digitalWrite(L_IN2, HIGH); ledcWrite(CH_L, -pwm); }
  else { digitalWrite(L_IN1, LOW); digitalWrite(L_IN2, LOW); ledcWrite(CH_L, 0); }
}

void motorR(int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm > 0) { digitalWrite(R_IN1, HIGH); digitalWrite(R_IN2, LOW); ledcWrite(CH_R, pwm); }
  else if (pwm < 0) { digitalWrite(R_IN1, LOW); digitalWrite(R_IN2, HIGH); ledcWrite(CH_R, -pwm); }
  else { digitalWrite(R_IN1, LOW); digitalWrite(R_IN2, LOW); ledcWrite(CH_R, 0); }
}

void motorsStop() { motorL(0); motorR(0); }

// ====== IMU ======
void imuBegin() {
  mpu.setWire(&Wire);
  mpu.beginAccel(); mpu.beginGyro(); mpu.beginMag();
  Serial.println("[IMU] Calibrating gyro bias...");
  double sum = 0;
  for (int i = 0; i < 300; i++) {
    mpu.gyroUpdate();
    sum += mpu.gyroZ();
    delay(5);
  }
  gyroBiasZ = sum / 300.0;
  yaw = 0;
  lastYawUpdate = millis();
  Serial.printf("[IMU] Bias Z = %.3f deg/s\n", gyroBiasZ);
}

// ====== Update Yaw ======
void updateYaw() {
  mpu.gyroUpdate();
  unsigned long now = millis();
  float dt = (now - lastYawUpdate) / 1000.0f;
  lastYawUpdate = now;
  yaw += (mpu.gyroZ() - gyroBiasZ) * dt;
  if (yaw > 180) yaw -= 360;
  if (yaw < -180) yaw += 360;
}

// ====== Angle Difference ======
float angleDiff(float target, float current) {
  float diff = target - current;
  while (diff > 180) diff -= 360;
  while (diff < -180) diff += 360;
  return diff;
}

// ====== Turn Until Target ======
void turnUntilTarget(float targetAngleDeg, int pwm) {
  Serial.println("\n↱ Turning until target...");
  float scaledTarget = targetAngleDeg / YAW_SCALE;   // compensate overshoot
  Serial.printf("Real target: %.2f° | Scaled target: %.2f°\n", targetAngleDeg, scaledTarget);

  while (true) {
    updateYaw();
    float error = angleDiff(scaledTarget, yaw);
    Serial.printf("Yaw: %7.2f | Target: %7.2f | Error: %7.2f | PWM: %3d\n",
                  yaw, scaledTarget, error, pwm);

    if (fabs(error) < 1.0) break;

    // turn right only (same direction each time)
    if (error > 0) { motorL(-pwm); motorR(pwm); }
    else { motorL(pwm); motorR(-pwm); }

    delay(10);
  }

  motorsStop();
  Serial.printf("✅ Done! Final Yaw: %.2f° (Target: %.2f°)\n", yaw, targetAngleDeg);
}

// ====== Setup ======
void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  Serial.println("=== Zeeker – Continuous 90° Turns ===");
  motorsBegin();
  imuBegin();
  delay(1000);
}

// ====== Loop: repeat 90° turns forever ======
void loop() {
  static float totalTarget = 0;   // keeps cumulative yaw target
  int TURN_SPEED = 70;

  totalTarget += 90.0f;          // add 90° each cycle
  if (totalTarget >= 360.0f) totalTarget -= 360.0f; // wrap after full circle

  turnUntilTarget(totalTarget, TURN_SPEED);
  delay(500); // pause between turns
}
