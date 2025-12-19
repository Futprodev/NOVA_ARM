#pragma once

#include <Arduino.h>
#include "Controller.hpp"
#include "JointsSimple.hpp"
#include "limits.hpp"
#include "config.hpp"

// ------------ Homing state ------------
struct HomingState {
  bool homing_base = true;
  bool homing_sh   = true;
  bool homing_el   = true;
  bool all_homed   = false;

  bool base_cleared = false;
  bool sh_cleared   = false;
  bool el_cleared   = false;
};

// ---------- Helper: limit switch ----------
inline bool isLimitActive(int pin) {
  // INPUT_PULLUP: pressed = LOW
  return digitalRead(pin) == LOW;
}

// ---------- Helper: open-loop homing motion ----------
inline void driveJointHoming(JointController &joint,
                             JointSimple &motor,
                             float home_speed_deg_s,
                             int home_dir_sign)
{
  float steps_per_joint_rev = joint.steps_per_motor_rev * joint.gear_ratio;
  float steps_s = (home_speed_deg_s / 360.0f) * steps_per_joint_rev;

  if (steps_s < 0.0f) steps_s = -steps_s;

  float freq = steps_s;
  if (freq < 1.0f) freq = 1.0f;
  freq = constrain(freq, 0.0f, joint.lim.max_freq_fast_hz);

  bool forward = (home_dir_sign > 0);
  if (joint.invert_dir) {
    forward = !forward;
  }

  motor.setDir(forward);
  motor.stepgen.setFrequency(freq);
}

inline void stopJointMotor(JointSimple &motor) {
  motor.stop();
}

// ---------- Helper: latch encoder at physical limit ----------
inline bool homeJointFromLimit(JointController &joint,
                               int limit_pin,
                               float hard_limit_deg,
                               const char *name)
{
  if (!isLimitActive(limit_pin)) {
    Serial.print(name);
    Serial.println(" limit NOT active, skipping.");
    return false;
  }

  if (!joint.homeAtLimit(hard_limit_deg)) {
    Serial.print(name);
    Serial.println(" encoder read FAILED during homing.");
    return false;
  }

  Serial.print(name);
  Serial.print(" homed at ");
  Serial.print(hard_limit_deg, 2);
  Serial.println(" deg (hard limit).");

  return true;
}

// ---------- Call this from setup() ----------
inline void initHomingState(HomingState &hs,
                            JointController &jointBase,
                            JointController &jointSh,
                            JointController &jointEl)
{
  // Teach current pose as zero (temporary)
  jointBase.teachHome();
  jointSh.teachHome();
  jointEl.teachHome();

  hs.homing_base  = true;
  hs.homing_sh    = false;
  hs.homing_el    = false;
  hs.all_homed    = false;
  hs.base_cleared = false;
  hs.sh_cleared   = false;
  hs.el_cleared   = false;

  Serial.println("Startup: automatic homing towards limit switches...");
}

// ---------- Call this from loop() while !hs.all_homed ----------
inline void updateHoming(HomingState &hs, float dt,
                         JointController &jointBase,
                         JointController &jointSh,
                         JointController &jointEl,
                         JointSimple &motorBase,
                         JointSimple &motorSh,
                         JointSimple &motorEl)
{
  (void)dt; // not used for now

  // ===== BASE =====
  if (hs.homing_base) {
    if (!hs.base_cleared) {
      if (isLimitActive(LIM_B)) {
        // still on switch → move AWAY
        driveJointHoming(jointBase, motorBase,
                         BASE_HOME_SPEED_DEG_S, -BASE_HOME_DIR);
      } else {
        hs.base_cleared = true;
        Serial.println("BASE: limit cleared, now approaching.");
        driveJointHoming(jointBase, motorBase,
                         BASE_HOME_SPEED_DEG_S, BASE_HOME_DIR);
      }
    } else {
      if (!isLimitActive(LIM_B)) {
        driveJointHoming(jointBase, motorBase,
                         BASE_HOME_SPEED_DEG_S, BASE_HOME_DIR);
      } else {
        stopJointMotor(motorBase);
        if (homeJointFromLimit(jointBase, LIM_B,
                               BASE_LIMITS.hard_max_deg,
                               "BASE")) {
          hs.homing_base = false;
          Serial.println("BASE homing done.");
          hs.homing_sh    = true;
          hs.sh_cleared   = false;
          hs.homing_el  = true;
          hs.el_cleared = false;
        }
      }
    }
  } else {
    stopJointMotor(motorBase);
  }

  // ===== SHOULDER =====
  if (hs.homing_sh) {
    if (!hs.sh_cleared) {
      if (isLimitActive(LIM_A)) {
        driveJointHoming(jointSh, motorSh,
                         SH_HOME_SPEED_DEG_S, -SH_HOME_DIR);
      } else {
        stopJointMotor(motorSh);
        hs.sh_cleared = true;
        Serial.println("SH: limit cleared, now approaching.");
      }
    } else {
      if (!isLimitActive(LIM_A)) {
        driveJointHoming(jointSh, motorSh,
                         SH_HOME_SPEED_DEG_S, SH_HOME_DIR);
      } else {
        stopJointMotor(motorSh);
        if (homeJointFromLimit(jointSh, LIM_A,
                               SH_LIMITS.hard_min_deg,
                               "SHOULDER")) {
          hs.homing_sh = false;
          Serial.println("SHOULDER homing done.");
        }
      }
    }
  } else {
    stopJointMotor(motorSh);
  }

  // ===== ELBOW =====
  if (hs.homing_el) {
    if (!hs.el_cleared) {
      if (isLimitActive(LIM_C)) {
        driveJointHoming(jointEl, motorEl,
                         EL_HOME_SPEED_DEG_S, -EL_HOME_DIR);
      } else {
        stopJointMotor(motorEl);
        hs.el_cleared = true;
        Serial.println("EL: limit cleared, now approaching.");
      }
    } else {
      if (!isLimitActive(LIM_C)) {
        driveJointHoming(jointEl, motorEl,
                         EL_HOME_SPEED_DEG_S, EL_HOME_DIR);
      } else {
        stopJointMotor(motorEl);
        if (homeJointFromLimit(jointEl, LIM_C,
                               EL_LIMITS.hard_min_deg,
                               "ELBOW")) {
          hs.homing_el = false;
          Serial.println("ELBOW homing done.");
        }
      }
    }
  } else {
    stopJointMotor(motorEl);
  }

  // ===== All done? =====
  if (!hs.homing_base && !hs.homing_sh && !hs.homing_el && !hs.all_homed) {
    hs.all_homed = true;
    Serial.println("=== ALL JOINTS HOMED ===");
  }
}
