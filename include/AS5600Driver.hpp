#pragma once
#include <Arduino.h>
#include <Wire.h>
#ifndef AS5600_DEBUG
#define AS5600_DEBUG 0
#endif

class AS5600Driver {
public:
    AS5600Driver(TwoWire &bus,
                 uint8_t as5600_addr,
                 uint8_t mux_addr,
                 uint8_t mux_channel,
                 bool invert = false,
                 float offset_deg = 0.0f)
    : bus_(bus),
      as5600_addr_(as5600_addr),
      mux_addr_(mux_addr),
      mux_ch_(mux_channel),
      invert_(invert),
      offset_deg_(offset_deg)
    {}

    void begin(uint32_t i2c_freq = 400000,
               int sda_pin = -1,
               int scl_pin = -1)
    {
        if (sda_pin >= 0 && scl_pin >= 0) {
            bus_.begin(sda_pin, scl_pin, i2c_freq);
        } else {
            bus_.begin();
        }
    }

    // Read raw 12-bit angle (0..4095) from AS5600
    bool readRaw(uint16_t &raw) {
    #if AS5600_DEBUG
        Serial.print("[AS5600] ch=");
        Serial.print(mux_ch_);
        Serial.println(" readRaw()...");
    #endif
        if (!selectMuxChannel()) {
    #if AS5600_DEBUG
            Serial.println("[AS5600] MUX select FAILED");
    #endif
            return false;
        }

        // ANGLE register high byte = 0x0E, low byte = 0x0F
        bus_.beginTransmission(as5600_addr_);
        bus_.write(0x0E);
        uint8_t err = bus_.endTransmission(false);  // repeated start
        if (err != 0) {
    #if AS5600_DEBUG
            Serial.print("[AS5600] TX angle reg FAILED, err=");
            Serial.println(err);
    #endif
            return false;
        }

        uint8_t n = bus_.requestFrom(as5600_addr_, (uint8_t)2);
        if (n < 2) {
    #if AS5600_DEBUG
            Serial.print("[AS5600] requestFrom got ");
            Serial.print(n);
            Serial.println(" bytes (need 2)");
    #endif
            return false;
        }

        uint8_t high = bus_.read();
        uint8_t low  = bus_.read();

        raw = ((uint16_t)(high & 0x0F) << 8) | low; // 12-bit

    #if AS5600_DEBUG
        Serial.print("[AS5600] raw=");
        Serial.println(raw);
    #endif
        return true;
    }

    // Read mechanical angle in degrees [0..360)
    bool readDeg(float &deg) {
        uint16_t raw;
        if (!readRaw(raw)) return false;

        float d = (float)raw * (360.0f / 4096.0f); // 0..360

        if (invert_) {
            d = 360.0f - d;
            if (d >= 360.0f) d -= 360.0f;
        }

        d += offset_deg_;
        while (d >= 360.0f) d -= 360.0f;
        while (d <  0.0f)   d += 360.0f;

        deg = d;
        return true;
    }

    bool readRad(float &rad) {
        float d;
        if (!readDeg(d)) return false;
        rad = d * DEG_TO_RAD;
        return true;
    }

private:
    bool selectMuxChannel() {
    #if AS5600_DEBUG
        Serial.print("[AS5600] select MUX ch=");
        Serial.print(mux_ch_);
        Serial.print(" addr=0x");
        Serial.println(mux_addr_, HEX);
    #endif

        bus_.beginTransmission(mux_addr_);
        bus_.write(1 << mux_ch_);
        uint8_t err = bus_.endTransmission();
        if (err != 0) {
    #if AS5600_DEBUG
            Serial.print("[AS5600] MUX endTransmission err=");
            Serial.println(err);
    #endif
            return false;
        }
        return true;
    }

    TwoWire &bus_;
    uint8_t as5600_addr_;
    uint8_t mux_addr_;
    uint8_t mux_ch_;
    bool    invert_;
    float   offset_deg_;
};