#pragma once
#include <algorithm>

struct SCurveState {
    float q = 0, qd = 0;
};

class SCurveLimiter {
    public:
        SCurveState step(float q, float qd, float qd_des, float a_max, float j_max, float dt) {
            float a_des = std::clamp((qd_des)/dt, -a_max, a_max);
            float da_max = j_max * dt;

            a_ = std::clamp(a_des, a_prev_ - da_max, a_prev_ + da_max);
            float qd_next = qd + a_ * dt;
            float q_next = q + qd_next * dt;
            a_prev_ = a_;
            return {q_next, qd_next};
        }

        void reset() {a_prev_ = a_ = 0;}
    
    private:
        float a_prev_ = 0, a_ = 0;
};