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

  // last good joint angle
  float last_q_deg   = 0.0f;
  bool  last_q_valid = false;

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
    if (!enc.readRad(rad)) {
      // I2C failed; if we have a last value, reuse it
      if (last_q_valid) {
        q_deg = last_q_deg;
        return true;
      }
      return false;
    }

    float raw_motor_deg = rad * 180.0f / PI;  // 0..360 from AS5600

    if (!unwrap_init) {
      // First valid sample: just initialize
      motor_prev_deg = raw_motor_deg;
      motor_turns    = 0;
      unwrap_init    = true;
    } else {
      float diff = raw_motor_deg - motor_prev_deg;
      float ad   = fabsf(diff);


      const float MAX_STEP_DEG = 100.0f;

      if (ad <= MAX_STEP_DEG) {
        // Normal small movement -> just accept
        motor_prev_deg = raw_motor_deg;
      }
      else if (ad >= (360.0f - MAX_STEP_DEG)) {
        if (diff < 0.0f) {
          motor_turns += 1;
        } else {
          motor_turns -= 1;
        }
        motor_prev_deg = raw_motor_deg;
      }
      else {
        if (last_q_valid) {
          q_deg = last_q_deg;   // reuse last good angle
          return true;
        } else {
          // no previous valid angle; we can't recover
          return false;
        }
      }
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
    motor_prev_deg   = raw_motor_deg;
    motor_turns      = 0;
    offset_motor_deg = raw_motor_deg;

    pid.reset();
    state.q  = 0.0f;
    state.qd = 0.0f;

    last_q_deg   = 0.0f;
    last_q_valid = true;
    return true;
  }

  // home at hard limit
  bool homeAtLimit(float joint_limit_deg) {
    float rad;
    if (!enc.readRad(rad)) return false;
    float raw_motor_deg = rad * 180.0f / PI;

    // Re-init unwrap from this pose
    unwrap_init    = false;
    motor_prev_deg = raw_motor_deg;
    motor_turns    = 0;

    // set offset so that current motor angle maps to desired joint limit
    offset_motor_deg = raw_motor_deg - joint_limit_deg * gear_ratio;

    pid.reset();
    state.q  = joint_limit_deg;
    state.qd = 0.0f;

    last_q_deg   = joint_limit_deg;
    last_q_valid = true;
    return true;
  }

  // main update: with dt in seconds
  void update(float dt) {
    float q_meas_deg;
    if (!readJointAngleDeg(q_meas_deg)) {

      // no encoder, no last value -> bail
      if (!last_q_valid) {
        motor.stop();
        return;
      }

      // q_meas_deg will be last_q_deg
    } else {
      last_q_deg   = q_meas_deg;
      last_q_valid = true;
    }

    // hard limit clamp
    if (q_meas_deg > lim.hard_max_deg) q_meas_deg = lim.soft_max_deg;
    if (q_meas_deg < lim.hard_min_deg) q_meas_deg = lim.soft_min_deg;

    // soft target clamp
    q_target_deg = constrain(q_target_deg,
                            lim.soft_min_deg,
                            lim.soft_max_deg);

    // Position error
    float e     = q_target_deg - q_meas_deg;
    float abs_e = fabsf(e);

    // Deadband: close enough -> stop
    const float deadband_deg = 0.2f;
    if (abs_e < deadband_deg) {
      motor.stop();
      pid.reset();
      state.qd = 0.0f;
      return;
    }

    // pick desired velocity from PID
    float qd_des;

    qd_des = pid.update(e, dt);

    // clamp by joint limits
    qd_des = constrain(qd_des,
                      -lim.max_deg_per_s,
                      +lim.max_deg_per_s);

    // s curve profile to reach desired velocity
    state = s_curve.step(q_meas_deg,
                        state.qd,
                        qd_des,
                        lim.max_deg_per_s2,
                        lim.max_deg_per_s3,
                        dt);

    // safety clamp on S-curve output
    state.qd = constrain(state.qd,
                        -lim.max_deg_per_s,
                        +lim.max_deg_per_s);

    // Convert joint vel to stepper freq
    float steps_per_joint_rev = steps_per_motor_rev * gear_ratio;
    float steps_s             = (state.qd / 360.0f) * steps_per_joint_rev;

    bool  forward = (steps_s >= 0.0f);
    float freq    = fabsf(steps_s);

    // Frequency cap depends on distance to target
    float freq_max = (abs_e > 10.0f)
                      ? lim.max_freq_fast_hz
                      : lim.max_freq_slow_hz;
    freq = constrain(freq, 0.0f, freq_max);

    // enforce a minimum usable freq if >0
    const float MIN_RUN_FREQ = 180.0f;
    if (freq > 0.0f && freq < MIN_RUN_FREQ) {
      freq = MIN_RUN_FREQ;
    }

    if (freq <= 0.0f) {
      motor.stop();
    } else {
      bool dir_cmd = forward;
      if (invert_dir) dir_cmd = !dir_cmd;
      motor.setDir(dir_cmd);
      motor.stepgen.setFrequency(freq);
    }
  }


};