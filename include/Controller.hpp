#pragma once

#include <Arduino.h>
#include <algorithm>      // for std::clamp
#include "AS5600Driver.hpp"
#include "JointsSimple.hpp"
#include "MotionProfile.hpp"
#include "limits.hpp"

// PID with anti-windup; gains will be filled from JointLimits
struct PIDAW {
  float kp = 0.0f, ki = 0.0f, kd = 0.0f;
  float i = 0.0f, prev_e = 0.0f;
  float i_min = -2.0f, i_max = 2.0f; // clamp integral

  float update(float e, float dt) {
    i = std::clamp(i + e * dt, i_min, i_max);
    float d = (e - prev_e) / dt;
    prev_e = e;
    return kp * e + ki * i + kd * d;
  }

  void reset() {
    i = 0.0f;
    prev_e = 0.0f;
  }
};

struct JointController {
  JointSimple&   motor;   // step/dir/enable + LEDC
  AS5600Driver&  enc;     // reads raw motor angle (rad)
  JointLimits    lim;     // per-joint config

  PIDAW          pid;     // angle -> desired vel (deg/s)
  SCurveLimiter  s_curve; // jerk-limited velocity profile
  SCurveState    state;   // q (deg), qd (deg/s)

  // gearing + steps
  float gear_ratio           = 16.0f;             // motor_turns : joint_turns
  float steps_per_motor_rev  = 200.0f * 16.0f;    // 200 steps * 16 microstep
  float offset_motor_deg     = 0.0f;              // home offset in motor deg

  // direction config
  bool  invert_dir = false;   // if true: flip motor direction

  // command
  float q_target_deg = 0.0f;

  // unwrap motor angle
  float motor_prev_deg = 0.0f;
  long  motor_turns    = 0;
  bool  unwrap_init    = false;

  // ---- constructor ----
  JointController(JointSimple& m,
                  AS5600Driver& e,
                  const JointLimits& l,
                  bool invert = false)
    : motor(m), enc(e), lim(l), invert_dir(invert)
  {
    // copy PID gains from limits
    pid.kp = lim.kp;
    pid.ki = lim.ki;
    pid.kd = lim.kd;
  }

  // ---- unwrap + gear + offset -> joint angle in deg ----
  bool readJointAngleDeg(float& q_deg) {
    float rad;
    if (!enc.readRad(rad)) return false;

    float raw_motor_deg = rad * 180.0f / PI;  // 0..360 motor

    // unwrap motor angle
    if (!unwrap_init) {
      motor_prev_deg = raw_motor_deg;
      motor_turns    = 0;
      unwrap_init    = true;
    } else {
      float diff = raw_motor_deg - motor_prev_deg;
      if (diff < -180.0f)      motor_turns += 1;  // 359 -> 0
      else if (diff > 180.0f)  motor_turns -= 1;  // 0 -> 359
      motor_prev_deg = raw_motor_deg;
    }

    float motor_deg_uw  = raw_motor_deg + 360.0f * (float)motor_turns;
    float motor_rel_deg = motor_deg_uw - offset_motor_deg; // apply home offset
    float joint_deg     = motor_rel_deg / gear_ratio;      // gearbox

    q_deg = joint_deg;
    return true;
  }

  // ---- teach current pose as zero ----
  bool teachHome() {
    float rad;
    if (!enc.readRad(rad)) return false;
    float raw_motor_deg = rad * 180.0f / PI;

    unwrap_init      = false;       // re-init unwrap from here
    offset_motor_deg = raw_motor_deg;
    pid.reset();
    state.q  = 0.0f;
    state.qd = 0.0f;
    return true;
  }

  // ---- periodic update: call at fixed dt (seconds) ----
  void update(float dt) {
    float q_meas_deg;
    if (!readJointAngleDeg(q_meas_deg)) {
      motor.stop();  // encoder failed – stop motor
      return;
    }

    // -------- HARD LIMITS: emergency clamp --------
    if (q_meas_deg <= lim.hard_min_deg ||
        q_meas_deg >= lim.hard_max_deg) {
      motor.stop();
      pid.reset();
      state.qd = 0.0f;
      // pull target back inside soft range
      q_target_deg = constrain(q_target_deg,
                               lim.soft_min_deg,
                               lim.soft_max_deg);
      return;
    }
    // ----------------------------------------------

    // Soft-clamp target
    float q_tgt_clamped = constrain(q_target_deg,
                                    lim.soft_min_deg,
                                    lim.soft_max_deg);

    // Position error
    float e = q_tgt_clamped - q_meas_deg;
    float abs_e = fabsf(e);

    // Deadband: close enough -> stop
    const float deadband_deg = 0.2f;
    if (abs_e < deadband_deg) {
      motor.stop();
      pid.reset();
      state.qd = 0.0f;
      return;
    }

    // PID: position error -> desired joint velocity (deg/s)
    float qd_des = pid.update(e, dt);
    qd_des = constrain(qd_des,
                       -lim.max_deg_per_s,
                       +lim.max_deg_per_s);

    // S-curve: limit acceleration & jerk
    state = s_curve.step(q_meas_deg,
                         state.qd,
                         qd_des,
                         lim.max_deg_per_s2,
                         lim.max_deg_per_s3,
                         dt);

    // Convert joint velocity (deg/s) -> step frequency
    float steps_per_joint_rev = steps_per_motor_rev * gear_ratio;
    float steps_s = (state.qd / 360.0f) * steps_per_joint_rev;

    bool  forward = (steps_s >= 0.0f);
    float freq   = fabsf(steps_s);

    // Frequency cap depends on how far we are from target
    float freq_max = (abs_e > 10.0f) ?  // >10° away → fast
                      lim.max_freq_fast_hz :
                      lim.max_freq_slow_hz;

    freq = constrain(freq, 0.0f, freq_max);

    if (freq < 1.0f) {
      motor.stop();
    } else {
      // use invert_dir to decide if we flip
      bool dir_cmd = forward;
      if (invert_dir) {
        dir_cmd = !dir_cmd;
      }
      motor.setDir(dir_cmd);
      motor.stepgen.setFrequency(freq);
    }
  }
};
