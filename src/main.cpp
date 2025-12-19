#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "config.hpp"
#include "stepGenLEDC.hpp"
#include "JointsSimple.hpp"
#include "AS5600Driver.hpp"
#include "limits.hpp"
#include "MotionProfile.hpp"
#include "Controller.hpp"
#include "Homing.hpp"

#include <ESP32Servo.h>

Servo effector;

// ---------------- I2C bus --------------------------
TwoWire I2C_BUS = TwoWire(0);

// ---------------- Motors ---------------------------
JointSimple motorBase(enB, dirB, stepB, LEDC_CH_B);  // BASE
JointSimple motorSh  (enA, dirA, stepA, LEDC_CH_A);  // SHOULDER
JointSimple motorEl  (enC, dirC, stepC, LEDC_CH_C);  // ELBOW

// ---------------- Encoders (via mux) ---------------
AS5600Driver encBase(I2C_BUS, AS5600_ADDR, I2C_MUX_ADDR, MUX_B, false, 0.0f);
AS5600Driver encSh  (I2C_BUS, AS5600_ADDR, I2C_MUX_ADDR, MUX_A, false, 0.0f);
AS5600Driver encEl  (I2C_BUS, AS5600_ADDR, I2C_MUX_ADDR, MUX_C, false, 0.0f);

// ---------------- Controllers ----------------------
JointController jointBase(motorBase, encBase, BASE_LIMITS, false);
JointController jointSh  (motorSh,   encSh,   SH_LIMITS,   false);
JointController jointEl  (motorEl,   encEl,   EL_LIMITS,   true);

// ---------------- Loop timing ----------------------
uint32_t last_us = 0;

bool  loopActive  = false;
bool  shGoingUp   = false;
bool  elGoingUp   = false;
bool  baseGoingUp = false;

// Smaller loop ranges (inside hard limits)
const float SH_MIN_LOOP   = 58.0f;
const float SH_MAX_LOOP   = 85.0f;

const float EL_MIN_LOOP   = -95.0f;
const float EL_MAX_LOOP   = -90.0f;

const float BASE_MIN_LOOP = -90.0f;
const float BASE_MAX_LOOP =  90.0f;

// Threshold for “we reached an endpoint”
const float LOOP_EPS = 2.0f;

// ---------------- Homing state ---------------------
HomingState homing;

// ---------------- Move sequence --------------------
enum MoveStage {
  MOVE_IDLE = 0,
  MOVE_SH_EL_TO_ZERO,
  MOVE_BASE_TO_TARGET,
  MOVE_SH_EL_TO_TARGET
};

struct MoveSequence {
  MoveStage stage = MOVE_IDLE;
  float target_base = 0.0f;
  float target_sh   = 0.0f;
  float target_el   = 0.0f;
  float hold_base   = 0.0f;  // base hold while SH/EL -> 0
};

MoveSequence moveSeq;

const float MOVE_TOL_DEG   = 0.5f;
const float REQ_DIFF_EPS   = 0.25f;

static inline bool diffGT(float a, float b, float eps) {
  return fabsf(a - b) > eps;
}

// Universal sequenced request
void requestTarget(float base_deg, float sh_deg, float el_deg, bool force=false)
{
  // clamp to soft limits
  float cb = constrain(base_deg, BASE_LIMITS.soft_min_deg, BASE_LIMITS.soft_max_deg);
  float cs = constrain(sh_deg,   SH_LIMITS.soft_min_deg,   SH_LIMITS.soft_max_deg);
  float ce = constrain(el_deg,   EL_LIMITS.soft_min_deg,   EL_LIMITS.soft_max_deg);

  // ignore trivial repeats unless forced
  if (!force) {
    if (!diffGT(cb, moveSeq.target_base, REQ_DIFF_EPS) &&
        !diffGT(cs, moveSeq.target_sh,   REQ_DIFF_EPS) &&
        !diffGT(ce, moveSeq.target_el,   REQ_DIFF_EPS)) {
      return;
    }
  }

  moveSeq.target_base = cb;
  moveSeq.target_sh   = cs;
  moveSeq.target_el   = ce;

  // latch base hold reference
  if (jointBase.last_q_valid) moveSeq.hold_base = jointBase.last_q_deg;
  else                        moveSeq.hold_base = jointBase.q_target_deg;

  moveSeq.stage = MOVE_SH_EL_TO_ZERO;

  Serial.print("[SEQ] Request: BASE=");
  Serial.print(moveSeq.target_base, 1);
  Serial.print(" SH=");
  Serial.print(moveSeq.target_sh, 1);
  Serial.print(" EL=");
  Serial.println(moveSeq.target_el, 1);
}

// ---------------- Loop goals -----------------------
float loopGoalBase = 0.0f;
float loopGoalSh   = 0.0f;
float loopGoalEl   = 0.0f;

// ---------------- Post-home edge detect ------------
static bool prev_all_homed = false;

// ---------------- SETUP ----------------------------
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);
  delay(300);

  // I2C
  I2C_BUS.begin(SDA_A, SCL_A, 100000);

  // Limit switches
  pinMode(LIM_A, INPUT_PULLUP);
  pinMode(LIM_B, INPUT_PULLUP);
  pinMode(LIM_C, INPUT_PULLUP);

  pinMode(servoPin, OUTPUT);
  effector.attach(servoPin);

  jointBase.gear_ratio = BASE_GEAR_RATIO;
  jointSh.gear_ratio   = 16.0f;
  jointEl.gear_ratio   = 16.0f;

  // Motors
  motorBase.begin();
  motorSh.begin();
  motorEl.begin();

  initHomingState(homing, jointBase, jointSh, jointEl);

  Serial.println("Startup: automatic homing towards limit switches...");

  last_us = micros();
}

// ---------------- SERIAL ---------------------------
void handleSerial() {
  if (!Serial.available()) return;

  char cmd = Serial.read();

  switch (cmd) {
    case 's':
    case 'S': {
      loopActive  = true;
      shGoingUp   = true;
      elGoingUp   = true;
      baseGoingUp = true;

      loopGoalSh   = SH_MAX_LOOP;
      loopGoalEl   = EL_MAX_LOOP;
      loopGoalBase = BASE_MAX_LOOP;

      requestTarget(loopGoalBase, loopGoalSh, loopGoalEl, true);

      Serial.println("[LOOP] Started (sequenced).");
      Serial.readStringUntil('\n');
      break;
    }

    case 'h':
    case 'H': {
      loopActive = false;

      // force a sequence to (0,0,0)
      requestTarget(0.0f, 0.0f, 0.0f, true);

      Serial.println("[CMD] Request -> (0,0,0) (sequenced).");
      Serial.readStringUntil('\n');
      break;
    }

    case 'p':
    case 'P': {
      if (jointBase.last_q_valid && jointSh.last_q_valid && jointEl.last_q_valid) {
        Serial.print("BASE=");
        Serial.print(jointBase.last_q_deg, 2);
        Serial.print("  SH=");
        Serial.print(jointSh.last_q_deg, 2);
        Serial.print("  EL=");
        Serial.println(jointEl.last_q_deg, 2);
      } else {
        Serial.println("Joint angles not valid yet (no successful reads).");
      }
      Serial.readStringUntil('\n');
      break;
    }

    default:
      Serial.readStringUntil('\n');
      Serial.println("Unknown cmd. Use 's' (loop), 'h' (home), 'p' (print).");
      break;
  }
}

// ---------------- MAIN LOOP ------------------------
void loop() {
  handleSerial();

  uint32_t now = micros();
  if (now - last_us >= (uint32_t)(CTRL_DT * 1e6f)) {
    float dt = (now - last_us) / 1e6f;
    last_us = now;

    if (!homing.all_homed) {
      // Open-loop homing phase
      updateHoming(homing, dt,
                   jointBase, jointSh, jointEl,
                   motorBase, motorSh, motorEl);

      prev_all_homed = false; // reset edge detect
    } else {

      // --------- FIRST tick after homing: HOLD pose ---------
      if (!prev_all_homed) {
        prev_all_homed = true;

        // Hold current pose (prevents any default 0 target jump)
        if (jointBase.last_q_valid) jointBase.q_target_deg = 0;
        if (jointSh.last_q_valid)   jointSh.q_target_deg   = jointSh.last_q_deg + 2;
        if (jointEl.last_q_valid)   jointEl.q_target_deg   = jointEl.last_q_deg + 2;

        Serial.println("[POSTHOME] Holding pose (no auto move).");
      }

      // ---------- sequence logic ----------
      if (moveSeq.stage != MOVE_IDLE) {

        // Always apply targets for each stage
        switch (moveSeq.stage) {

          case MOVE_SH_EL_TO_ZERO:
            jointSh.q_target_deg   = 0.0f;
            jointEl.q_target_deg   = 0.0f;
            jointBase.q_target_deg = moveSeq.hold_base;
            break;

          case MOVE_BASE_TO_TARGET:
            jointBase.q_target_deg = moveSeq.target_base;
            jointSh.q_target_deg   = 0.0f;
            jointEl.q_target_deg   = 0.0f;
            break;

          case MOVE_SH_EL_TO_TARGET:
            jointBase.q_target_deg = moveSeq.target_base;
            jointSh.q_target_deg   = moveSeq.target_sh;
            jointEl.q_target_deg   = moveSeq.target_el;
            break;

          default:
            break;
        }

        // Advance stages only when we have valid angles
        if (jointBase.last_q_valid && jointSh.last_q_valid && jointEl.last_q_valid) {
          float q_ba = jointBase.last_q_deg;
          float q_sh = jointSh.last_q_deg;
          float q_el = jointEl.last_q_deg;

          if (moveSeq.stage == MOVE_SH_EL_TO_ZERO) {
            if (fabsf(q_sh) < MOVE_TOL_DEG && fabsf(q_el) < MOVE_TOL_DEG) {
              moveSeq.stage = MOVE_BASE_TO_TARGET;
              Serial.println("[SEQ] SH/EL at 0 -> moving BASE");
            }
          }
          else if (moveSeq.stage == MOVE_BASE_TO_TARGET) {
            if (fabsf(q_ba - moveSeq.target_base) < MOVE_TOL_DEG) {
              moveSeq.stage = MOVE_SH_EL_TO_TARGET;
              Serial.println("[SEQ] BASE at target -> moving SH/EL");
            }
          }
          else if (moveSeq.stage == MOVE_SH_EL_TO_TARGET) {
            if (fabsf(q_sh - moveSeq.target_sh) < MOVE_TOL_DEG &&
                fabsf(q_el - moveSeq.target_el) < MOVE_TOL_DEG) {
              moveSeq.stage = MOVE_IDLE;
              delay(100);
              effector.write(0);
              delay(1000);
              effector.write(145);
              Serial.println("[SEQ] Complete");
            }
          }
        }
      }

      // ---------- update controllers -----------
      jointBase.update(dt);
      jointSh.update(dt);
      jointEl.update(dt);

      // ---------- loop active: check for endpoint and reverse --------
      if (loopActive &&
          moveSeq.stage == MOVE_IDLE &&
          jointBase.last_q_valid &&
          jointSh.last_q_valid &&
          jointEl.last_q_valid) {

        float q_ba = jointBase.last_q_deg;
        float q_sh = jointSh.last_q_deg;
        float q_el = jointEl.last_q_deg;

        bool changed = false;

        if (shGoingUp) {
          if (q_sh > (SH_MAX_LOOP - LOOP_EPS)) {
            shGoingUp = false;
            loopGoalSh = SH_MIN_LOOP;
            changed = true;
          }
        } else {
          if (q_sh < (SH_MIN_LOOP + LOOP_EPS)) {
            shGoingUp = true;
            loopGoalSh = SH_MAX_LOOP;
            changed = true;
          }
        }

        if (elGoingUp) {
          if (q_el > (EL_MAX_LOOP - LOOP_EPS)) {
            elGoingUp = false;
            loopGoalEl = EL_MIN_LOOP;
            changed = true;
          }
        } else {
          if (q_el < (EL_MIN_LOOP + LOOP_EPS)) {
            elGoingUp = true;
            loopGoalEl = EL_MAX_LOOP;
            changed = true;
          }
        }

        if (baseGoingUp) {
          if (q_ba > (BASE_MAX_LOOP - LOOP_EPS)) {
            baseGoingUp = false;
            loopGoalBase = BASE_MIN_LOOP;
            changed = true;
          }
        } else {
          if (q_ba < (BASE_MIN_LOOP + LOOP_EPS)) {
            baseGoingUp = true;
            loopGoalBase = BASE_MAX_LOOP;
            changed = true;
          }
        }

        if (changed) {
          requestTarget(loopGoalBase, loopGoalSh, loopGoalEl, true);
        }
      }
    }

    // ---- periodic debug ----
    static uint32_t last_print = 0;
    if (millis() - last_print > 200) {
      Serial.print("BA q=");
      Serial.print(jointBase.last_q_valid ? jointBase.last_q_deg : NAN, 1);
      Serial.print(" tgt=");
      Serial.print(jointBase.q_target_deg, 1);
      Serial.print(" | ");

      Serial.print("SH q=");
      Serial.print(jointSh.last_q_valid ? jointSh.last_q_deg : NAN, 1);
      Serial.print(" tgt=");
      Serial.print(jointSh.q_target_deg, 1);
      Serial.print(" | ");

      Serial.print("EL q=");
      Serial.print(jointEl.last_q_valid ? jointEl.last_q_deg : NAN, 1);
      Serial.print(" tgt=");
      Serial.print(jointEl.q_target_deg, 1);

      if (loopActive) Serial.print("  [LOOP]");
      if (moveSeq.stage != MOVE_IDLE) {
        Serial.print("  [SEQ STAGE=");
        Serial.print((int)moveSeq.stage);
        Serial.print("]");
      }
      Serial.println();

      last_print = millis();
    }
  }
}
