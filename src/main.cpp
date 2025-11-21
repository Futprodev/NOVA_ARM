#include <Arduino.h>
#include <Wire.h>
#include "pins.hpp"
#include "JointsSimple.hpp"
#include "AS5600Driver.hpp"

// -------- TwoWire buses for your encoders --------
// TwoWire I2C_A = TwoWire(0);  // Motor A encoder bus
// TwoWire I2C_C = TwoWire(1);  // Motor C encoder bus

//AS5600Driver encA(I2C_A, AS5600_ADDR, false, 0.0f);
AS5600Driver encC(Wire, AS5600_ADDR, false, 0.0f);

// -------- Joints (match your pins, with LEDC) -----
JointSimple jointA(enA, dirA, stepA, LEDC_CH_A);
JointSimple jointB(enB, dirB, stepB, LEDC_CH_B);
JointSimple jointC(enC, dirC, stepC, LEDC_CH_C);

// -------- FreeRTOS tasks (encoder printouts) ------
//TaskHandle_t TaskEncoderA;
TaskHandle_t TaskEncoderC;

// void taskEncoderA(void*) {
//   for (;;) {
//     float rad;
//     if (encA.readRad(rad)) {
//       float deg = rad * 180.0f / PI;
//       Serial.print("EncA: "); Serial.println(deg, 2);
//     } else {
//       Serial.println("EncA: read fail");
//     }
//     vTaskDelay(pdMS_TO_TICKS(50));  
//   }
// }

void taskEncoderC(void*) {
  for (;;) {
    float rad;
    if (encC.readRad(rad)) {
      float deg = rad * 180.0f / PI;
      Serial.print("EncC: "); Serial.println(deg, 2);
    } else {
      Serial.println("EncC: read fail");
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

static void handleCommand(char cmd) {
  switch (cmd) {
    // ----- MOTOR A (1.7A - ARM) -----
    case '1': jointA.setSpeedFromDelayUs(600); Serial.println("Motor A speed: SLOW");   break;
    case '2': jointA.setSpeedFromDelayUs(400); Serial.println("Motor A speed: NORMAL"); break;
    case '3': jointA.stop();                    Serial.println("Motor A: STOP");         break;
    case 'a': jointA.setDir(false);             Serial.println("Motor A LEFT (CCW)");    break;
    case 'd': jointA.setDir(true);              Serial.println("Motor A RIGHT (CW)");    break;

    // ----- MOTOR B (1.3A - Axis 360) -----
    case '4': jointB.setSpeedFromDelayUs(600);  Serial.println("Motor B speed: SLOW");   break;
    case '5': jointB.setSpeedFromDelayUs(400);  Serial.println("Motor B speed: NORMAL"); break;
    case '6': jointB.stop();                    Serial.println("Motor B: STOP");         break;
    case 'o': jointB.setDir(false);             Serial.println("Motor B LEFT (CCW)");    break;
    case 'p': jointB.setDir(true);              Serial.println("Motor B RIGHT (CW)");    break;

    // ----- MOTOR C (0.7A - Small) -----
    case '7': jointC.setSpeedFromDelayUs(600);  Serial.println("Motor C speed: SLOW");   break;
    case '8': jointC.setSpeedFromDelayUs(400);  Serial.println("Motor C speed: NORMAL"); break;
    case '9': jointC.stop();                    Serial.println("Motor C: STOP");         break;
    case 'n': jointC.setDir(false);             Serial.println("Motor C LEFT (CCW)");    break;
    case 'm': jointC.setDir(true);              Serial.println("Motor C RIGHT (CW)");    break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);

  // Encoders on your two buses
  //encA.begin(SDA_A, SCL_A, 400000);
  Wire.begin(SDA_C, SCL_C, 400000);
  encC.begin(SDA_C, SCL_C, 400000);

  // Joints
  jointA.begin();
  jointB.begin();
  jointC.begin();

  Serial.println("=== 3 MOTOR CONTROL (LEDC + AS5600, ESP32-S3) ===");
  Serial.println("A: 1=Slow, 2=Normal, 3=Stop, Dir=a/d");
  Serial.println("B: 4=Slow, 5=Normal, 6=Stop, Dir=o/p");
  Serial.println("C: 7=Slow, 8=Normal, 9=Stop, Dir=n/m");

  // Encoder print tasks (Core 0; motors run in hardware via LEDC)
  //xTaskCreatePinnedToCore(taskEncoderA, "EncA", 2048, nullptr, 1, &TaskEncoderA, 0);
  xTaskCreatePinnedToCore(taskEncoderC, "EncC", 2048, nullptr, 1, &TaskEncoderC, 0);
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    handleCommand(c);
  }
  vTaskDelay(pdMS_TO_TICKS(10));
}
