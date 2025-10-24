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
#define ENCA1 35  // Left encoder channel A
#define ENCA2 34  // Left encoder channel B
#define ENCB1 33  // Right encoder channel A
#define ENCB2 32  // Right encoder channel B

// ====== Buzzer ======
#define BUZZER_PIN 27
#define BUZZ_CH    7
bool BUZZ_ENABLED = true;
static unsigned long buzz_end_ms = 0;

// ====== Globals ======
Adafruit_VL6180X tof;
MPU9250_asukiaaa mpu;

// Encoder variables
volatile int posA = 0;           // Left ticks
volatile int posB = 0;           // Right ticks
volatile int lastEncodedA = 0;   // last AB snapshot (left)
volatile int lastEncodedB = 0;   // last AB snapshot (right)

// ====== IMU Globals ======
float yaw = 0;
float gyroBiasZ = 0;
unsigned long lastYawUpdate = 0;

// ====== Calibration ======
struct Cal { float a, b; };
Cal CAL[] = {
  {1.00f, -42.0f},  // LEFT
  {1.00f,  20.0f},  // FRONT-LEFT
  {1.00f, -35.0f},  // FRONT
  {1.00f, -10.0f},  // FRONT-RIGHT
  {1.00f, -28.0f}   // RIGHT
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

// ====== MUX Helper ======
void mux(uint8_t ch) {
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
  delayMicroseconds(200);
}

// ====== ToF Read ======
float readMM(uint8_t ch, int idx) {
  mux(ch);
  delay(2);
  uint8_t r = tof.readRange();
  if (tof.readRangeStatus() != VL6180X_ERROR_NONE) return -1.0f;
  float mm = CAL[idx].a * r + CAL[idx].b;
  if (mm < 0) mm = 0;
  return mm;
}

// ====== IMU Setup + Gyro Bias Calibration ======
void imuBegin() {
  mpu.setWire(&Wire);
  mpu.beginAccel();
  mpu.beginGyro();
  mpu.beginMag();

  Serial.println("[IMU] Initializing...");
  delay(1200);

  // --- Calibrate gyro bias (keep robot still) ---
  Serial.println("[IMU] Calibrating gyro bias, keep still...");
  double sum = 0;
  int samples = 0;
  for (int i = 0; i < 200; i++) {
    mpu.gyroUpdate();
    sum += mpu.gyroZ();
    samples++;
    delay(5);
  }
  gyroBiasZ = (samples > 0) ? sum / samples : 0;
  Serial.printf("[IMU] Gyro bias Z = %.3f deg/s\n", gyroBiasZ);

  yaw = 0;
  lastYawUpdate = millis();

  Serial.println("[IMU] Ready.\n");
}

// ====== Buzzer Functions ======
void buzzerBegin() {
  pinMode(BUZZER_PIN, OUTPUT);
  ledcSetup(BUZZ_CH, 2000 /*Hz*/, 8 /*bits*/);
  ledcAttachPin(BUZZER_PIN, BUZZ_CH);
  ledcWriteTone(BUZZ_CH, 0); // off
}

void buzzStart(int freqHz, int durMs) {
  if (!BUZZ_ENABLED) return;
  ledcWriteTone(BUZZ_CH, freqHz);
  buzz_end_ms = millis() + durMs;
}

void buzzUpdate() {
  if (buzz_end_ms != 0 && (long)(millis() - buzz_end_ms) >= 0) {
    ledcWriteTone(BUZZ_CH, 0);
    buzz_end_ms = 0;
  }
}

void buzzOK()        { buzzStart(1500, 80); }    // short chirp
void buzzAttention() { buzzStart(500, 150); }    // low alert
void buzzDoubleOK()  { buzzStart(1500, 80); delay(120); buzzStart(1800, 80); }
void buzzSuccess() {
  buzzStart(800, 120);
  delay(150);
  buzzStart(1000, 120);
  delay(150);
  buzzStart(1200, 180);
}

// ====== Setup ======
void setup() {
  Serial.begin(115200);
  delay(500);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  Serial.println("\n=== Zeeker ToF + IMU + Encoder Diagnostic ===");

  // ToF setup
  uint8_t channels[5] = {CH_FRONT_LEFT, CH_FRONT_CENTER, CH_FRONT_RIGHT, CH_LEFT_SIDE, CH_RIGHT_SIDE};
  for (int i = 0; i < 5; i++) {
    mux(channels[i]);
    delay(3);
    if (tof.begin(&Wire))
      Serial.printf("✅ ToF[%d] ready on CH%d\n", i, channels[i]);
    else
      Serial.printf("❌ ToF[%d] FAIL on CH%d\n", i, channels[i]);
  }

  // IMU setup
  imuBegin();
  Serial.println("✅ MPU9250 ready.\n");

  // Encoders
  pinMode(ENCA1, INPUT_PULLUP);
  pinMode(ENCA2, INPUT_PULLUP);
  pinMode(ENCB1, INPUT_PULLUP);
  pinMode(ENCB2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCA1), updateEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCA2), updateEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB1), updateEncoderB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB2), updateEncoderB, CHANGE);

  Serial.println("✅ Encoders initialized.\n");

  // Buzzer
  buzzerBegin();
  buzzDoubleOK(); // Startup sound
  Serial.println("✅ Buzzer ready.\n");
}

// ====== Loop ======
void loop() {
  // --- ToF readings ---
  float L  = readMM(CH_LEFT_SIDE, 0);
  float FL = readMM(CH_FRONT_LEFT, 1);
  float F  = readMM(CH_FRONT_CENTER, 2);
  float FR = readMM(CH_FRONT_RIGHT, 3);
  float R  = readMM(CH_RIGHT_SIDE, 4);

  // --- IMU yaw integration ---
  mpu.gyroUpdate();
  unsigned long now = millis();
  float dt = (now - lastYawUpdate) / 1000.0f;
  lastYawUpdate = now;

  float gz = mpu.gyroZ() - gyroBiasZ;
  yaw += gz * dt;
  if (yaw > 180) yaw -= 360;
  if (yaw < -180) yaw += 360;

  // --- Print diagnostics ---
  noInterrupts();
  long leftTicks = posA;
  long rightTicks = posB;
  interrupts();

  Serial.printf("Yaw:%7.2f° | L:%4.0f FL:%4.0f F:%4.0f FR:%4.0f R:%4.0f | ENC_L:%6ld ENC_R:%6ld\n",
                yaw, L, FL, F, FR, R, leftTicks, rightTicks);

  buzzUpdate();
  delay(10);
}
