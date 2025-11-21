#pragma once
#include <Arduino.h>

// Simple STEP generator using new ESP32 Arduino 3.x LEDC API.
// Uses "pin-based" attach instead of channels.
//
// - begin()   : attach LEDC to the given pin
// - setFrequency(hz):
//       hz > 0  -> square wave at 'hz' with ~50% duty
//       hz <= 0 -> stop pulses (duty = 0)
class StepGenLEDC {
public:
  // 'ledc_ch' is kept only so old code compiles, but it's ignored.
  StepGenLEDC(uint8_t pin, uint8_t /*ledc_ch*/ = 0,
              uint8_t resolution_bits = 8)
    : pin_(pin), res_(resolution_bits), attached_(false) {}

  void begin(uint32_t init_freq = 1000) {
    // Attach LEDC to this pin at some initial frequency.
    // New API: ledcAttach(pin, freq, resolution_bits)
    attached_ = ledcAttach(pin_, init_freq, res_);
    if (attached_) {
      ledcWrite(pin_, 0); // start with no pulses
    }
  }

  void setFrequency(float freq_hz) {
    if (!attached_) return;

    if (freq_hz <= 0.0f) {
      // stop pulses
      ledcWrite(pin_, 0);
      return;
    }

    // New API: change freq per pin
    uint32_t f = (uint32_t)freq_hz;
    ledcChangeFrequency(pin_, f, res_);

    uint32_t maxDuty = (1U << res_) - 1U;
    ledcWrite(pin_, maxDuty / 2); // ~50% duty square wave
  }

private:
  uint8_t pin_;
  uint8_t res_;
  bool attached_;
};
