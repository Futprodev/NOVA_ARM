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
// Bus 0: shoulder encoder (Motor A, pins SDA_A/SCL_A)
// Bus 1: elbow encoder   (Motor C, pins SDA_C/SCL_C)
TwoWire I2C_SH(0);
TwoWire I2C_EL(1);

// ---------------- Motors -----------------------------
JointSimple motorSh(enA, dirA, stepA, LEDC_CH_A);  // SHOULDER on motor A pins
JointSimple motorEl(enC, dirC, stepC, LEDC_CH_C);  // ELBOW on motor C pins

// ---------------- Encoders ---------------------------
AS5600Driver encSh(I2C_SH, AS5600_ADDR, false,  0.0f);  // shoulder encoder
AS5600Driver encEl(I2C_EL, AS5600_ADDR, false, 0.0f);  // elbow encoder

// ---------------- Controllers ------------------------
// last bool = invert_dir flag
JointController jointSh(motorSh, encSh, SH_LIMITS, false);
JointController jointEl(motorEl, encEl, EL_LIMITS, true);

// ---- Loop timing ----
uint32_t last_us = 0;

// ---- Auto loop state ----
bool loopActive   = false;
bool shGoingUp    = true;   // SH: 0 -> 50 -> 0 -> 50 ...
bool elGoingUp    = true;   // EL: 120 -> -120 -> 120 ...

// desired segment limits
const float SH_MIN_LOOP = -20.0f;
const float SH_MAX_LOOP = 20.0f;

const float EL_MIN_LOOP = -120.0f;
const float EL_MAX_LOOP =  120.0f;

// threshold for "we reached the end" (deg)
const float LOOP_EPS = 2.0f;

// =====================================================

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);
  delay(300);

  // I2C init for encoders
  I2C_SH.begin(SDA_A, SCL_A, 400000);  // shoulder encoder (Motor A)
  I2C_EL.begin(SDA_C, SCL_C, 400000);  // elbow encoder   (Motor C)

  // Motor + controller init
  motorSh.begin();
  motorEl.begin();

  // --- AUTO-HOME BOTH JOINTS ON STARTUP ---
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
  Serial.println("  s  -> start SH(0↔50) + EL(120↔-120) loop");
  Serial.println("  h  -> stop loop and go to (0,0)");

  last_us = micros();
}

// =====================================================

void handleSerial() {
  if (!Serial.available()) return;

  char cmd = Serial.read();

  switch (cmd) {
    // ---- START LOOP ----
    case 's':
    case 'S': {
      loopActive = true;

      // start from one end: SH up towards 50, EL up towards 120
      shGoingUp = true;
      elGoingUp = true;

      jointSh.q_target_deg = SH_MAX_LOOP;
      jointEl.q_target_deg = EL_MAX_LOOP;

      Serial.println("Loop START: SH 0↔50, EL 120↔-120.");
      // flush rest of line
      Serial.readStringUntil('\n');
      break;
    }

    // ---- STOP LOOP + GO TO (0,0) ----
    case 'h':
    case 'H': {
      loopActive = false;

      jointSh.q_target_deg = 0.0f;
      jointEl.q_target_deg = 0.0f;

      Serial.println("Loop STOP: targets -> SH=0, EL=0");
      Serial.readStringUntil('\n');
      break;
    }

    default:
      // flush the rest of the line so nothing weird is left
      Serial.readStringUntil('\n');
      Serial.println("Unknown cmd. Use: s (start loop), h (stop + home 0,0).");
      break;
  }
}

// =====================================================

void loop() {
  handleSerial();

  uint32_t now = micros();
  if (now - last_us >= (uint32_t)(CTRL_DT * 1e6f)) {
    float dt = (now - last_us) / 1e6f;
    last_us = now;

    // --- If loop active, manage end-point flipping ---
    if (loopActive) {
      float q_sh, q_el;
      if (jointSh.readJointAngleDeg(q_sh) && jointEl.readJointAngleDeg(q_el)) {

        // SHOULDER: 0 <-> 50
        if (shGoingUp) {
          // going towards 50
          if (q_sh > (SH_MAX_LOOP - LOOP_EPS)) {
            shGoingUp = false;
            jointSh.q_target_deg = SH_MIN_LOOP;   // go back down to 0
          }
        } else {
          // going towards 0
          if (q_sh < (SH_MIN_LOOP + LOOP_EPS)) {
            shGoingUp = true;
            jointSh.q_target_deg = SH_MAX_LOOP;   // go up to 50
          }
        }

        // ELBOW: 120 <-> -120
        if (elGoingUp) {
          // going towards +120
          if (q_el > (EL_MAX_LOOP - LOOP_EPS)) {
            elGoingUp = false;
            jointEl.q_target_deg = EL_MIN_LOOP;   // go down to -120
          }
        } else {
          // going towards -120
          if (q_el < (EL_MIN_LOOP + LOOP_EPS)) {
            elGoingUp = true;
            jointEl.q_target_deg = EL_MAX_LOOP;   // go up to 120
          }
        }
      }
    }

    // --- Update both joints (control + S-curve + LEDC) ---
    jointSh.update(dt);
    jointEl.update(dt);

    // Optional debug
    static uint32_t last_print = 0;
    if (millis() - last_print > 200) {
      float q_sh, q_el;

      if (jointSh.readJointAngleDeg(q_sh)) {
        Serial.print("SH q=");
        Serial.print(q_sh, 1);
        Serial.print(" tgt=");
        Serial.print(jointSh.q_target_deg, 1);
        Serial.print(" | ");
      }

      if (jointEl.readJointAngleDeg(q_el)) {
        Serial.print("EL q=");
        Serial.print(q_el, 1);
        Serial.print(" tgt=");
        Serial.print(jointEl.q_target_deg, 1);
      }

      if (loopActive) Serial.print("  [LOOP]");
      Serial.println();

      last_print = millis();
    }
  }
}