#pragma once
struct JointLimits {
    float soft_min_deg;
    float soft_max_deg;
    float max_deg_per_s;
    float max_deg_per_s2;
    float max_deg_per_s3;
};

inline JointLimits BASE_LIMITS { -160, 160, 90, 180, 500 };
inline JointLimits SH_LIMITS   { -80,  80, 90, 180, 500 };
inline JointLimits EL_LIMITS   { -120,120, 90, 180, 500 };
