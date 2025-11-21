#pragma once
#include <cstdint>

enum class JointIdx : uint8_t { BASE=0, SHOULDER=1, ELBOW=2, COUNT=3};

struct JointState {
    float q_deg = 0, qd_deg_s = 0;  // data from hall sensor, angle and vel
    float q_cmd_deg = 0;            // command for position
    float qd_cmd_deg_s = 0;         // controller output vel
};