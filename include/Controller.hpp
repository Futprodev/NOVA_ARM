#pragma once
#include <algorithm>

struct PIDAW { //anti windup
    float kp = 4, ki = 1, kd = 0.1;
    float i = 0, prev_e = 0;
    float i_min = -2, i_max = 2; //clamp integral

    float update (float e, float dt) {
        i = std::clamp(i + e*dt, i_min, i_max);
        float d = (e - prev_e)/dt;
        prev_e = e;
        return kp*e + ki*i + kd*d;
    }
};