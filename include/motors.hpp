#pragma once

// 17HS3401, 17HS4401, 17HS4023
struct MotorSpec {
  const char* name;
  float rated_current_A;   // phase RMS (or convert from A/phase; see TMC note below)
  int   full_steps;        // 200 for 1.8°
  int   microsteps;        // 16 default (TMC can interpolate to 256)
  float gear_ratio;        // 1.0 if none
  float max_mech_rps;      // conservative mechanical RPM/60 (e.g., 2 rps = 120 rpm)
};

inline MotorSpec BASE_MOTOR {1.2f, 200, 16, 1.0f};  // slight underdrive is fine
inline MotorSpec SH_MOTOR   {1.4f, 200, 16, 1.0f};
inline MotorSpec EL_MOTOR   {0.6f, 200, 16, 1.0f};

