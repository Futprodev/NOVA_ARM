#pragma once
#include <Arduino.h>
#include "StepGenLEDC.hpp"

struct JointSimple {
  // wiring
  int pin_en, pin_dir;
  StepGenLEDC stepgen;

  // runtime state
  bool  enabled      = false;
  bool  dir_fwd      = true;
  float steps_per_s  = 0.0f; // current step rate

  // Keep the 'ledc_ch' parameter so existing code compiles,
  // but internally we ignore it (StepGenLEDC handles LEDC by pin).
  JointSimple(int en, int dir, int step, int ledc_ch)
  : pin_en(en), pin_dir(dir), stepgen(step, ledc_ch) {}

  void begin() {
    pinMode(pin_en, OUTPUT);
    pinMode(pin_dir, OUTPUT);
    enable(true);
    setDir(true);
    stepgen.begin();  // attach LEDC to STEP pin
  }

  void enable(bool en) {
    // Your hardware: EN = LOW => enabled
    digitalWrite(pin_en, en ? LOW : HIGH);
    enabled = en;
    if (!en) {
      stepgen.setFrequency(0);
    }
  }

  void setDir(bool forward) {
    digitalWrite(pin_dir, forward ? HIGH : LOW);
    dir_fwd = forward;
  }

  // Same mapping as your old delayMicroseconds-based code:
  //  delay = 400 µs -> 1/(2*400e-6)  = 1250 steps/s
  //  delay = 600 µs -> 1/(2*600e-6)  ≈ 833.3 steps/s
  void setSpeedFromDelayUs(int delay_us) {
    if (delay_us <= 0) {
      stepgen.setFrequency(0);
      steps_per_s = 0.0f;
      return;
    }
    float hz = 1e6f / (2.0f * (float)delay_us);
    steps_per_s = hz;
    stepgen.setFrequency(hz);
  }

  void stop() {
    setSpeedFromDelayUs(0);
  }
};
