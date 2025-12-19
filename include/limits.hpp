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

// Limit Switch Base: 152, Shoulder: -94, Elbow: -142
// ===================== ELBOW (light, fastest) =====================
inline const JointLimits EL_LIMITS {
  .soft_min_deg      = -141.0f,
  .soft_max_deg      =  156.7f,
  .hard_min_deg      = -143.0f,
  .hard_max_deg      =  157.0f,

  .max_deg_per_s     = 20.0f,   // was 5
  .max_deg_per_s2    = 200.0f,  // was 50
  .max_deg_per_s3    = 800.0f,  // was 200

  .max_freq_fast_hz  = 2800.0f, // was 850
  .max_freq_slow_hz  = 1400.0f,  // was 400

  .kp = 0.3f, .ki = 0.0f, .kd = 0.1f
};

// ===================== SHOULDER (medium) =====================
inline const JointLimits SH_LIMITS {
  .soft_min_deg      = -94.0f,
  .soft_max_deg      =  104.7f,
  .hard_min_deg      = -94.0f,
  .hard_max_deg      =  105.0f,

  .max_deg_per_s     = 20.0f,   // was 5
  .max_deg_per_s2    = 150.0f,  // was 50
  .max_deg_per_s3    = 600.0f,  // was 200

  .max_freq_fast_hz  = 2800.0f, // was 850
  .max_freq_slow_hz  = 1400.0f,  // was 400

  .kp = 0.3f, .ki = 0.0f, .kd = 0.11f
};

// ===================== BASE (heaviest, slowest) =====================
inline const JointLimits BASE_LIMITS {
  .soft_min_deg      = -150.0f,
  .soft_max_deg      =  150.0f,
  .hard_min_deg      = -152.0f,
  .hard_max_deg      =  152.0f,

  // --- SPEED LIMITS ---
  .max_deg_per_s     = 20.0f,     
  .max_deg_per_s2    = 100.0f,   
  .max_deg_per_s3    = 300.0f,   

  .max_freq_fast_hz  = 1400.0f,  
  .max_freq_slow_hz  = 711.0f,   

  .kp = 0.06f, .ki = 0.0f, .kd = 0.0f
};