#pragma once

struct JointLimits {
  // Angle limits (degrees)
  float soft_min_deg;     // normal target range
  float soft_max_deg;
  float hard_min_deg;     // absolute safety clamp
  float hard_max_deg;

  // Motion limits (deg/s, deg/s^2, deg/s^3)
  float max_deg_per_s;
  float max_deg_per_s2;
  float max_deg_per_s3;

  // Step frequency caps (Hz) for LEDC
  float max_freq_fast_hz;   // far from target
  float max_freq_slow_hz;   // near target

  // PID gains (angle -> velocity)
  float kp, ki, kd;
};

// ===================== ELBOW (light, fastest) =====================
inline const JointLimits EL_LIMITS {
  .soft_min_deg      = -150.0f,
  .soft_max_deg      =  150.0f,
  .hard_min_deg      = -152.0f,
  .hard_max_deg      =  152.0f,

  .max_deg_per_s     = 200.0f,
  .max_deg_per_s2    = 1500.0f,
  .max_deg_per_s3    = 6000.0f,

  .max_freq_fast_hz  = 6250.0f,   //8250
  .max_freq_slow_hz  = 2250.0f,   //2250

  .kp = 0.0025f, .ki = 0.0f, .kd = 0.0039f
  // 0.017, 0.0, 0.018
};

// ===================== SHOULDER (medium) =====================
inline const JointLimits SH_LIMITS {
  .soft_min_deg      = -90.0f,
  .soft_max_deg      =  90.0f,
  .hard_min_deg      = -105.0f,
  .hard_max_deg      =  105.0f,

  .max_deg_per_s     = 14.0f, //140
  .max_deg_per_s2    = 50.0f, //1000
  .max_deg_per_s3    = 200.0f, //4000

  .max_freq_fast_hz  = 5250.0f, //5250
  .max_freq_slow_hz  = 2250.0f, //1250

  .kp = 0.0022f, .ki = 0.0f, .kd = 0.0037f
  // 0.01, 0.0, 0.012
};

// ===================== BASE (heaviest, slowest) =====================
inline const JointLimits BASE_LIMITS {
  .soft_min_deg      = -90.0f,
  .soft_max_deg      =  90.0f,
  .hard_min_deg      = -95.0f,
  .hard_max_deg      =  95.0f,

  .max_deg_per_s     = 90.0f,
  .max_deg_per_s2    = 800.0f,
  .max_deg_per_s3    = 3000.0f,

  .max_freq_fast_hz  = 5000.0f,
  .max_freq_slow_hz  = 2000.0f,

  .kp = 0.3f, .ki = 0.0f, .kd = 0.002f
};