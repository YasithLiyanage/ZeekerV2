// #include <Wire.h>
// #include <Arduino.h>
// #include "Adafruit_VL6180X.h"

// // ====== I²C and MUX Pins ======
// #define SDA_PIN 26
// #define SCL_PIN 25
// #define MUX_ADDR 0x70

// // ====== Sensor Channels ======
// #define CH_FL   4    // Front-left 45°
// #define CH_FC   5    // Front-center
// #define CH_FR   6    // Front-right 45°
// #define CH_LS   3    // Left side
// #define CH_RS   7    // Right side

// // ====== Global ======
// Adafruit_VL6180X tof;

// struct Cal { float a, b; };
// Cal CAL[5];  // FL, FC, FR, LS, RS

// // ====== MUX SELECT ======
// void mux(uint8_t ch) {
//   Wire.beginTransmission(MUX_ADDR);
//   Wire.write(1 << ch);
//   Wire.endTransmission();
//   delayMicroseconds(300);
// }

// // ====== RAW READ (no calibration) ======
// float readRaw(uint8_t ch) {
//   mux(ch);
//   delay(3);
//   uint8_t r = tof.readRange();
//   if (tof.readRangeStatus() != VL6180X_ERROR_NONE)
//     return -1.0f;
//   return (float)r;
// }

// // ====== Sensor Calibration Routine ======
// void calibrateSensor(uint8_t ch, const char* name, int index)
// {
//   Serial.printf("\n========== Calibrating %s ==========\n", name);

//   float raw20, raw60, raw100;

//   // ---- 20 mm ----
//   Serial.printf("Place %s exactly at 20mm then press any key\n", name);
//   while (!Serial.available()) delay(10);
//   Serial.read();
//   raw20 = readRaw(ch);
//   Serial.printf("raw20 = %.2f\n", raw20);

//   // ---- 60 mm ----
//   Serial.printf("Place %s at 60mm then press any key\n", name);
//   while (!Serial.available()) delay(10);
//   Serial.read();
//   raw60 = readRaw(ch);
//   Serial.printf("raw60 = %.2f\n", raw60);

//   // ---- 100 mm ----
//   Serial.printf("Place %s at 100mm then press any key\n", name);
//   while (!Serial.available()) delay(10);
//   Serial.read();
//   raw100 = readRaw(ch);
//   Serial.printf("raw100 = %.2f\n", raw100);

//   // ----- Linear fit -----
//   float mm1 = 20.0f, mm2 = 100.0f;
//   float a = (mm2 - mm1) / (raw100 - raw20);
//   float b = mm1 - a * raw20;

//   CAL[index].a = a;
//   CAL[index].b = b;

//   Serial.printf("DONE %s → a=%.4f   b=%.4f\n", name, a, b);
// }

// // ====== Master Calibration ======
// void calibrateAllSensors()
// {
//   Serial.println("\n\n===============================");
//   Serial.println("📏 START FULL SENSOR CALIBRATION");
//   Serial.println("===============================\n");

//   calibrateSensor(CH_FL, "Front-Left (45°)", 0);
//   calibrateSensor(CH_FC, "Front-Center",      1);
//   calibrateSensor(CH_FR, "Front-Right (45°)", 2);
//   calibrateSensor(CH_LS, "Left-Side",         3);
//   calibrateSensor(CH_RS, "Right-Side",        4);

//   Serial.println("\n===== FINAL CAL VALUES =====");
//   for (int i = 0; i < 5; i++) {
//     Serial.printf("{ %.4f, %.4f },\n", CAL[i].a, CAL[i].b);
//   }

//   Serial.println("\nSave these values back into CAL[] in your main micromouse code.");
// }

// // ====== Setup ======
// void setup() {
//   Serial.begin(115200);
//   delay(500);

//   Wire.begin(SDA_PIN, SCL_PIN);
//   Wire.setClock(400000);

//   // Init ToF on each mux channel
//   uint8_t CH[5] = { CH_FL, CH_FC, CH_FR, CH_LS, CH_RS };
//   for (int i = 0; i < 5; i++) {
//     mux(CH[i]);
//     delay(5);
//     tof.begin(&Wire);
//   }

//   delay(500);
//   calibrateAllSensors();

//   // Stop forever after calibration
//   while (1) delay(1000);
// }

// // ====== LOOP ======
// void loop() {}
