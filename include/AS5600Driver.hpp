#pragma once
#include <Arduino.h>
#include <Wire.h>

// Lightweight AS5600 reader bound to a given TwoWire bus.
// Reads ANGLE (0x0E/0x0F) -> [0, 2π) radians.
class AS5600Driver {
public:
  AS5600Driver(TwoWire& bus, uint8_t addr = 0x36, bool invert=false, float zero_offset_rad=0.0f)
  : bus_(bus), addr_(addr), invert_(invert), zero_(zero_offset_rad) {}

  void begin(int, int, uint32_t) {
    // nothing – bus already initialized in setup
  }

  bool readRad(float& rad) {
    bus_.beginTransmission(addr_);
    bus_.write((uint8_t)0x0E);
    uint8_t err = bus_.endTransmission();
    if (err != 0) {
      Serial.print("AS5600 I2C error (endTx), addr=0x");
      Serial.print(addr_, HEX);
      Serial.print(" code=");
      Serial.println(err);
      return false;
    }

    int n = bus_.requestFrom(addr_, (uint8_t)2);
    if (n != 2) {
      Serial.print("AS5600 requestFrom fail, addr=0x");
      Serial.print(addr_, HEX);
      Serial.print(" bytes=");
      Serial.println(n);
      return false;
    }

    uint16_t high = bus_.read();
    uint16_t low  = bus_.read();
    uint16_t raw  = ((high << 8) | low) & 0x0FFF;

    float a = (float)raw * (2.0f * PI / 4096.0f);
    rad = a;
    return true;
  }


private:
  TwoWire& bus_;
  uint8_t addr_;
  bool invert_;
  float zero_;
};
