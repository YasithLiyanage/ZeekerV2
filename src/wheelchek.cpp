// #include <Arduino.h>

// // ====== Motor Wiring ======
// // Left motor
// #define L_IN1 5
// #define L_IN2 17
// #define L_PWM 18  // PWM pin for left motor

// // Right motor
// #define R_IN1 2
// #define R_IN2 4
// #define R_PWM 15   // PWM pin for right motor

// // ====== PWM Channels ======
// #define CH_L 0
// #define CH_R 1


// // ====== Drive Function ======
// void drive(String dir, int speed) {
//   // Constrain PWM value
//   speed = constrain(speed, 0, 255);

//   if (dir == "forward") {
//     digitalWrite(L_IN1, HIGH);
//     digitalWrite(L_IN2, LOW);
//     digitalWrite(R_IN1, HIGH);
//     digitalWrite(R_IN2, LOW);
//   }
//   else if (dir == "backward") {
//     digitalWrite(L_IN1, LOW);
//     digitalWrite(L_IN2, HIGH);
//     digitalWrite(R_IN1, LOW);
//     digitalWrite(R_IN2, HIGH);
//   }
//   else if (dir == "left") {
//     digitalWrite(L_IN1, LOW);
//     digitalWrite(L_IN2, HIGH);
//     digitalWrite(R_IN1, HIGH);
//     digitalWrite(R_IN2, LOW);
//   }
//   else if (dir == "right") {
//     digitalWrite(L_IN1, HIGH);
//     digitalWrite(L_IN2, LOW);
//     digitalWrite(R_IN1, LOW);
//     digitalWrite(R_IN2, HIGH);
//   }
//   else if (dir == "stop") {
//     digitalWrite(L_IN1, LOW);
//     digitalWrite(L_IN2, LOW);
//     digitalWrite(R_IN1, LOW);
//     digitalWrite(R_IN2, LOW);
//   }

//   // Apply speed via PWM
//   ledcWrite(CH_L, speed);
//   ledcWrite(CH_R, speed);
// }

// // ====== Setup ======
// void setup() {
//   // Direction pins
//   pinMode(L_IN1, OUTPUT);
//   pinMode(L_IN2, OUTPUT);
//   pinMode(R_IN1, OUTPUT);
//   pinMode(R_IN2, OUTPUT);

//   // PWM setup
//   ledcSetup(CH_L, 1000, 8); // 1 kHz, 8-bit resolution
//   ledcSetup(CH_R, 1000, 8);
//   ledcAttachPin(L_PWM, CH_L);
//   ledcAttachPin(R_PWM, CH_R);

//   Serial.begin(115200);
//   Serial.println("Motor driver ready!");

//   // Example: move forward at medium speed
//   drive("forward", 180);
// }



// // ====== Loop ======
// void loop() {
//   // Example motion pattern
//   drive("forward", 180);
//   delay(2000);

//   drive("left", 180);
//   delay(1000);

//   drive("right", 180);
//   delay(1000);

//   drive("backward", 180);
//   delay(2000);

//   drive("stop", 0);
//   delay(1000);
// }
