#pragma once
// ---------------- Motor pins ----------------
constexpr int enA   = 4;
constexpr int stepA = 5;
constexpr int dirA  = 6;

constexpr int enB   = 15;
constexpr int stepB = 16;
constexpr int dirB  = 17;

constexpr int enC   = 9;
constexpr int stepC = 10;
constexpr int dirC  = 11;

// ---------------- Servo Pin ---------------------
constexpr int SERVOPIN = 1;

// ---------------- LEDC channels -----------------
constexpr int LEDC_CH_A = 0;
constexpr int LEDC_CH_B = 1;
constexpr int LEDC_CH_C = 2;

// ---------------- AS5600 / I2C buses --------------------------
constexpr uint8_t AS5600_ADDR = 0x36;

// Enc Shoulder or Main MUX
constexpr int SDA_A = 20;
constexpr int SCL_A = 21;

// Enc Elbow
constexpr int SDA_C = 2;
constexpr int SCL_C = 1;

// Enc Base
constexpr int SDA_B = 13;
constexpr int SCL_B = 12;

constexpr int MUX_A = 6;
constexpr int MUX_B = 5;
constexpr int MUX_C = 7;

// --------------- Limit --------------------u
constexpr int LIM_A = 37;
constexpr int LIM_B = 36;
constexpr int LIM_C = 47;

// MUX
constexpr uint8_t I2C_MUX_ADDR = 0x70;

// timing
constexpr float CTRL_HZ = 100.0f;
constexpr float CTRL_DT = 1.0f / CTRL_HZ;

constexpr float BASE_GEAR_RATIO = 4.0f;

constexpr int BASE_HOME_DIR = -1;  // +1 = one direction, -1 = opposite
constexpr int SH_HOME_DIR   = +1;
constexpr int EL_HOME_DIR   = +1;

constexpr float BASE_HOME_SPEED_DEG_S = 15.0f;
constexpr float SH_HOME_SPEED_DEG_S   = 10.0f;
constexpr float EL_HOME_SPEED_DEG_S   = 15.0f;