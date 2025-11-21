#pragma once

// ---------------- Motor pins (match your test) ----------------
constexpr int enA   = 4;
constexpr int stepA = 5;
constexpr int dirA  = 6;

constexpr int enB   = 15;
constexpr int stepB = 16;
constexpr int dirB  = 17;

constexpr int enC   = 12;
constexpr int stepC = 10;
constexpr int dirC  = 11;

// ---------------- LEDC channels (3 channels) -----------------
constexpr int LEDC_CH_A = 0;
constexpr int LEDC_CH_B = 1;
constexpr int LEDC_CH_C = 2;

// ---------------- AS5600 / I2C buses --------------------------
constexpr uint8_t AS5600_ADDR = 0x36;

// // Bus 0 (your "Motor A" encoder)
// constexpr int SDA_A = 21;
// constexpr int SCL_A = 20;   // consider using a safer pin than 20

// Bus 1 (your "Motor C" encoder)
constexpr int SDA_C = 8;
constexpr int SCL_C = 9;
