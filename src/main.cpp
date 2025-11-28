#include <Arduino.h>
#include <Wire.h>

#include "config.hpp"
#include "stepGenLEDC.hpp"
#include "JointsSimple.hpp"
#include "AS5600Driver.hpp"
#include "limits.hpp"
#include "MotionProfile.hpp"
#include "Controller.hpp"

// ---------------- I2C buses --------------------------
TwoWire I2C_SH(0);  // SHOULDER encoder bus
TwoWire I2C_EL(1);  // ELBOW encoder bus

// ---------------- Motors -----------------------------
JointSimple motorSh(enA, dirA, stepA, LEDC_CH_A);
JointSimple motorEl(enC, dirC, stepC, LEDC_CH_C);

// ---------------- Encoders ---------------------------
AS5600Driver encSh(I2C_SH, AS5600_ADDR, false, 0.0f);
AS5600Driver encEl(I2C_EL, AS5600_ADDR, false, 0.0f);

// ---------------- Controllers ------------------------
JointController jointSh(motorSh, encSh, SH_LIMITS, false);  // invert_dir=false
JointController jointEl(motorEl, encEl, EL_LIMITS, true);   // invert_dir=true

// ---- Loop timing ----
uint32_t last_us = 0;

// Forward declaration so handleSerial can call it
void printJointStates();

// =====================================================
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);
  delay(300);

  // I2C init for encoders
  I2C_SH.begin(SDA_A, SCL_A, 400000);
  I2C_EL.begin(SDA_C, SCL_C, 400000);

  // Motor + controller init
  motorSh.begin();
  motorEl.begin();

  if (jointSh.teachHome()) {
    Serial.println("Startup: SHOULDER pose set as 0 deg (home).");
  } else {
    Serial.println("Startup: SHOULDER home FAILED (encoder).");
  }

  if (jointEl.teachHome()) {
    Serial.println("Startup: ELBOW pose set as 0 deg (home).");
  } else {
    Serial.println("Startup: ELBOW home FAILED (encoder).");
  }

  jointSh.q_target_deg = 0.0f;
  jointEl.q_target_deg = 0.0f;

  Serial.println("Joint controllers ready.");
  Serial.println("Commands:");
  Serial.println("  S <deg>  -> move SHOULDER smoothly to angle");
  Serial.println("  E <deg>  -> move ELBOW smoothly to angle");
  Serial.println("  H        -> set BOTH joints home at current pose");
  Serial.println("  Q        -> print SH=...,EL=...");

  last_us = micros();

  Serial.println("READY");
}


// =====================================================
void handleSerial() {
  if (!Serial.available()) return;

  char cmd = Serial.read();

  switch (cmd) {
    case 'S':
    case 's': {
      float a = Serial.parseFloat();
      // flush rest of line (terminator) so next cmd starts clean
      Serial.readStringUntil('\n');

      a = constrain(a, SH_LIMITS.soft_min_deg, SH_LIMITS.soft_max_deg);
      jointSh.q_target_deg = a;
      Serial.print("Shoulder tgt = ");
      Serial.println(jointSh.q_target_deg);
      break;
    }

    case 'E':
    case 'e': {
      float a = Serial.parseFloat();
      Serial.readStringUntil('\n');

      a = constrain(a, EL_LIMITS.soft_min_deg, EL_LIMITS.soft_max_deg);
      jointEl.q_target_deg = a;
      Serial.print("Elbow tgt = ");
      Serial.println(jointEl.q_target_deg);
      break;
    }

    case 'H':
    case 'h': {
      Serial.readStringUntil('\n');  // just eat rest if user typed "H\n"
      bool okSh = jointSh.teachHome();
      bool okEl = jointEl.teachHome();
      Serial.print("Home set. SH=");
      Serial.print(okSh ? "OK" : "FAIL");
      Serial.print(" EL=");
      Serial.println(okEl ? "OK" : "FAIL");
      break;
    }

    case 'Q':
    case 'q': {
      Serial.readStringUntil('\n');  // flush newline if present
      printJointStates();
      break;
    }

    default:
      Serial.readStringUntil('\n'); // flush unknown line
      Serial.println("Unknown cmd. Use: S/E <deg>, H, Q");
      break;
  }
}

// Helper to print current joint angles in a parseable format
void printJointStates()
{
  float q_sh, q_el;
  bool okSh = jointSh.readJointAngleDeg(q_sh);
  bool okEl = jointEl.readJointAngleDeg(q_el);

  Serial.print("SH=");
  if (okSh) Serial.print(q_sh, 3); else Serial.print("NaN");
  Serial.print(",EL=");
  if (okEl) Serial.print(q_el, 3); else Serial.print("NaN");
  Serial.println();
}

// =====================================================
void loop() {
  handleSerial();

  uint32_t now = micros();
  if (now - last_us >= (uint32_t)(CTRL_DT * 1e6f)) {
    float dt = (now - last_us) / 1e6f;
    last_us = now;

    jointSh.update(dt);
    jointEl.update(dt);
  }
}