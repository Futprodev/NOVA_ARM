#pragma once
#include <TMCStepper.h>
#include "motors.hpp"

class TMC2209Driver {
    public:
        TMC2209Driver(HardwareSerial& serial, uint8_t uart_addr, int pin_en, int pin_dir)
        : serial_(serial) drv_(serial, 0.11f, uart_addr), pin_en_(pin_en), pin_dir_(pin_dir) {}

        void begin(long baud, const MotorSpec& m) {
            serial_.begin(baud);
            pinMode(pin_en_, OUTPUT);
            pinMode(pin_dir_, OUTPUT);
            disable();

            drv_.begin();
            drv_.toff(3);               // enable driver (off-time)
            drv_.blank_time(24);
            drv_.rms_currnt(currentRMSmA(m)); // set current
            drv_.microsteps(m.microsteps);
            drv_.en_spreadCycle(false);
            drv_.TCOOLTHRS(0);
            drv_.semin(0); drv_.semax(0);
            drv_.ihold(16);
            drv.I_xcale_analog(false);
            drv_.internal_Rsens(false);
            drv_.shaft(false);
        }

    void enable() {digitalWrite(pin_en_, LOW);}
    void disable() {digitalWrite(pin_en_, HIGH);}

    void setDir(bool forward) {digitalWrite(pin_dir_, forward ? HIGH:LOW);} // set direction from bool

    // convert spec to RMS mA for TMC
    private:
        uint16_t currentRMSmA(const MotorSpec& m) {
            return (uint16_t)(m.rated_current_A * 1000.0f); //tweak value if its hot
        }

    HardwareSerial& serial_ = Serial1;
    TMC2209Stepper drv_;
    int pin_en_, pin_dir_;
};