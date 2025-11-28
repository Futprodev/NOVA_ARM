#pragma once
#include <Arduino.h>

class StepGenLEDC {
public:
    StepGenLEDC(uint8_t pin, int8_t channel, uint8_t resolution_bits = 8)
      : pin_(pin), chan_(channel), res_(resolution_bits), attached_(false) {}

    void begin(uint32_t init_freq = 1000) {
        // Attach this pin to an LEDC channel with initial freq + resolution
        attached_ = ledcAttachChannel(pin_, init_freq, res_, chan_);
        if (!attached_) {
            Serial.print("[StepGenLEDC] ledcAttachChannel FAILED on pin ");
            Serial.println(pin_);
            return;
        }

        // Start with no pulses
        ledcWrite(pin_, 0);
        Serial.print("[StepGenLEDC] attached pin ");
        Serial.print(pin_);
        Serial.print(" to channel ");
        Serial.print(chan_);
        Serial.print(" at ");
        Serial.print(init_freq);
        Serial.println(" Hz");
    }

    void setFrequency(float freq_hz) {
        if (!attached_) return;

        if (freq_hz <= 0.0f) {
            // stop pulses
            ledcWrite(pin_, 0);
            return;
        }

        // Safety limits
        if (freq_hz > 15000.0f) freq_hz = 15000.0f;
        if (freq_hz < 1.0f)     freq_hz = 1.0f;

        bool ok = ledcChangeFrequency(pin_, (uint32_t)freq_hz, res_);
        if (!ok) {
            Serial.print("[StepGenLEDC] ledcChangeFrequency FAILED at ");
            Serial.print(freq_hz);
            Serial.println(" Hz");
            return;
        }

        uint32_t maxDuty = (1U << res_) - 1U;
        ledcWrite(pin_, maxDuty / 2); // ~50% duty square wave
    }

private:
    uint8_t pin_;
    int8_t  chan_;
    uint8_t res_;
    bool    attached_;
};
