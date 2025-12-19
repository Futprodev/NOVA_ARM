#include <Arduino.h>
#include <Wire.h>

#include "config.hpp"
#include "stepGenLEDC.hpp"
#include "JointsSimple.hpp"
#include "AS5600Driver.hpp"
#include "limits.hpp"
#include "MotionProfile.hpp"
#include "Controller.hpp"

// --------------- Servo ----------------------------
#include <ESP32Servo.h>
bool was_opened = false;
Servo effector;

// --------------- Homing ---------------------------
#include "Homing.hpp"
HomingState homing;

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

// ---- Loop timing ----
uint32_t last_us = 0;

void printJointStates();

// Sequence
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

const float BASE_SKIP_ZERO_EPS = 1.0f;
static bool prev_all_homed = false;
const float MOVE_TOL_DEG   = 0.5f;
const float REQ_DIFF_EPS   = 0.25f;

static inline bool diffGT(float a, float b, float eps) {
  return fabsf(a - b) > eps;
}

// Universal sequenced request
void requestTarget(float base_deg, float sh_deg, float el_deg, bool force=false)
{
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

  // current/known base angle (best available)
  float base_now = (jointBase.last_q_valid) ? jointBase.last_q_deg : jointBase.q_target_deg;

  moveSeq.target_base = cb;
  moveSeq.target_sh   = cs;
  moveSeq.target_el   = ce;

  // latch base hold reference (always)
  moveSeq.hold_base = base_now;

  float base_delta = fabsf(cb - base_now);

  // ---- NEW LOGIC: if base doesn't need to move, skip SH/EL->0 ----
  if (base_delta <= BASE_SKIP_ZERO_EPS) {
    moveSeq.stage = MOVE_SH_EL_TO_TARGET;  // direct move
    Serial.print("[SEQ] Base unchanged (|d|=");
    Serial.print(base_delta, 2);
    Serial.println(") -> direct SH/EL");
  } else {
    moveSeq.stage = MOVE_SH_EL_TO_ZERO;    // do the safe sequence
    Serial.print("[SEQ] Base moving (|d|=");
    Serial.print(base_delta, 2);
    Serial.println(") -> SH/EL->0 then BASE");
  }

  Serial.print("[SEQ] Request: BASE=");
  Serial.print(moveSeq.target_base, 1);
  Serial.print(" SH=");
  Serial.print(moveSeq.target_sh, 1);
  Serial.print(" EL=");
  Serial.println(moveSeq.target_el, 1);
}

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

  pinMode(SERVOPIN, OUTPUT);
  effector.attach(SERVOPIN);

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

void handleSerial() {
  if (!Serial.available()) return;

  char cmd = Serial.read();

  switch (cmd) {
    case 'T':
    case 't': {
      float b = Serial.parseFloat();
      float s = Serial.parseFloat();
      float e = Serial.parseFloat();
      Serial.readStringUntil('\n'); // flush rest of line

      requestTarget(b, s, e, true); // force = true (always run sequence)
      Serial.println("[CMD] T accepted (sequenced).");
      break;
    }

    case 'G':
    case 'g': {
      int st = Serial.parseInt();    // 0 or 1
      Serial.readStringUntil('\n');  // flush

      if (st <= 0) {
        was_opened = false;
        effector.write(0);           // CLOSE
      } else {
        was_opened = true;
        effector.write(145);         // OPEN
      }

      Serial.print("[GRIP] set ");
      Serial.println(was_opened ? "OPEN" : "CLOSE");
      break;
    }


    case 'H':
    case 'h': {
      Serial.readStringUntil('\n');
      requestTarget(0.0f, 0.0f, 0.0f, true);
      Serial.println("[CMD] Home request -> (0,0,0) (sequenced).");
      break;
    }

    case 'Q':
    case 'q': {
      Serial.readStringUntil('\n');
      printJointStates();
      break;
    }

    default:
      Serial.readStringUntil('\n');
      Serial.println("Unknown cmd. Use: T b s e | B/S/E deg | H | Q");
      break;
  }
}

// Helper to print current joint angles in a parseable format
void printJointStates()
{
  float q_base, q_sh, q_el;
  bool okBase = jointBase.readJointAngleDeg(q_base);
  bool okSh = jointSh.readJointAngleDeg(q_sh);
  bool okEl = jointEl.readJointAngleDeg(q_el);

  Serial.print("BS=");
  if (okBase) Serial.print(q_base, 3); else Serial.print("NaN");
  Serial.print("SH=");
  if (okSh) Serial.print(q_sh, 3); else Serial.print("NaN");
  Serial.print(",EL=");
  if (okEl) Serial.print(q_el, 3); else Serial.print("NaN");
  Serial.println();
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
      // --------- FIRST tick after homing: HOLD pose with some margin ---------
      if (!prev_all_homed) {
        prev_all_homed = true;

        // Hold current pose (prevents any default 0 target jump)
        if (jointBase.last_q_valid) jointBase.q_target_deg = 0;
        if (jointSh.last_q_valid)   jointSh.q_target_deg   = jointSh.last_q_deg + 2;
        if (jointEl.last_q_valid)   jointEl.q_target_deg   = jointEl.last_q_deg + 2;

        Serial.println("[POSTHOME] Holding pose (no auto move).");
      }

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
              Serial.println("SEQ_DONE");
            }
          }
        }
      }

      jointBase.update(dt);
      jointSh.update(dt);
      jointEl.update(dt);
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
      Serial.println(jointEl.q_target_deg, 1);
    }
  }
}