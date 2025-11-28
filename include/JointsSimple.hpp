#pragma once
#include <Arduino.h>
#include "StepGenLEDC.hpp"

struct JointSimple {
  int pin_en, pin_dir;
  StepGenLEDC stepgen;

  bool  enabled     = false;
  bool  dir_fwd     = true;
  float steps_per_s = 0.0f;

  JointSimple(int en, int dir, int step, int ledc_ch)
    : pin_en(en), pin_dir(dir), stepgen(step, ledc_ch, 8) {}

  void begin() {
    pinMode(pin_en, OUTPUT);
    pinMode(pin_dir, OUTPUT);
    enable(true);
    setDir(true);
    stepgen.begin();  
  }

  void enable(bool en) {
    // EN = LOW => enabled (for your drivers)
    digitalWrite(pin_en, en ? LOW : HIGH);
    enabled = en;
    if (!en) stepgen.setFrequency(0);
  }

  void setDir(bool forward) {
    digitalWrite(pin_dir, forward ? HIGH : LOW);
    dir_fwd = forward;
  }

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

  void stop() { setSpeedFromDelayUs(0); }
};
