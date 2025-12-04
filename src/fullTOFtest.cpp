// #include <Wire.h>
// #include <Arduino.h>
// #include "Adafruit_VL6180X.h"

// // ====== I²C and MUX Pins ======
// #define SDA_PIN 26
// #define SCL_PIN 25
// #define MUX_ADDR 0x70

// // ====== MUX Channels ======
// #define CH_FL   4
// #define CH_FC   5
// #define CH_FR   6
// #define CH_LS   3
// #define CH_RS   7

// // ====== ToF ======
// Adafruit_VL6180X tof;

// // ====== Calibration (YOUR VALUES) ======
// struct Cal { float a, b; };
// Cal CAL[5] = {
//   { 0.9524f,  19.0476f },   // FL
//   { 1.0000f, -36.0000f },   // FC
//   { 0.9756f,  -4.3902f },   // FR
//   { 1.0667f, -49.3333f },   // LS
//   { 1.0811f, -29.7297f }    // RS
// };

// // Channel map index → CAL index
// uint8_t CH_LIST[5] = { CH_FL, CH_FC, CH_FR, CH_LS, CH_RS };

// // ====== MUX select ======
// void mux(uint8_t ch) {
//   Wire.beginTransmission(MUX_ADDR);
//   Wire.write(1 << ch);
//   Wire.endTransmission();
//   delayMicroseconds(300);
// }

// // ====== Read RAW ======
// float readRaw(uint8_t ch) {
//   mux(ch);
//   delay(3);
//   uint8_t r = tof.readRange();

//   if (tof.readRangeStatus() != VL6180X_ERROR_NONE)
//     return -1.0f;

//   return (float)r;
// }

// // ====== Read calibrated distance (mm) ======
// float readMM(uint8_t ch, int idx) {
//   float raw = readRaw(ch);
//   if (raw < 0) return -1;
//   float mm = CAL[idx].a * raw + CAL[idx].b;
//   if (mm < 0) mm = 0;
//   return mm;
// }

// // ====== Setup ======
// void setup() {
//   Serial.begin(115200);
//   delay(500);

//   Wire.begin(SDA_PIN, SCL_PIN);
//   Wire.setClock(400000);

//   // init sensors on each MUX channel
//   for (int i = 0; i < 5; i++) {
//     mux(CH_LIST[i]);
//     delay(5);
//     tof.begin(&Wire);
//   }

//   Serial.println("\n===== SENSOR TEST START =====");
//   Serial.println("FL | FC | FR | LS | RS");
// }

// // ====== Main Loop ======
// void loop() {
//   float FL = readMM(CH_FL, 0);
//   float FC = readMM(CH_FC, 1);
//   float FR = readMM(CH_FR, 2);
//   float LS = readMM(CH_LS, 3);
//   float RS = readMM(CH_RS, 4);

//   Serial.printf("FL:%5.1f  FC:%5.1f  FR:%5.1f   LS:%5.1f  RS:%5.1f\n",
//                  FL,       FC,       FR,         LS,      RS);

//   delay(100);
// }
