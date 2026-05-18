// MotorDriver.h
// kontrol motr drivernya
#pragma once

#include <Arduino.h>
#include "Types.h"
#include "Config.h"

class MotorDriver {
public:
    MotorDriver();

    // Initialize pins and PWM channels
    void begin();

    // Set wheel speeds in rad/s. Speeds are converted to PWM using
    // scaling relative to max wheel speed and max PWM.
    void setWheelSpeed(const WheelSpeed &speed);

    // Stop all motors
    void stop();

private:
    struct MotorConfig {
        int in1;
        int in2;
        int pwm;
        int pwmChannel;
    };

    MotorConfig _fl;
    MotorConfig _fr;
    MotorConfig _rl;
    MotorConfig _rr;

    void setSingleMotor(const MotorConfig &mc, float radPerSec);
};