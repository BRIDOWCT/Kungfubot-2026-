// MotorDriver.cpp
// implementasi kontrol mototr driver
#include "MotorDriver.h"
#include <Arduino.h>

MotorDriver::MotorDriver() {
    // Configure each motor's pins and PWM channel
    _fl = {Config::FL_IN1_PIN, Config::FL_IN2_PIN, Config::FL_PWM_PIN, 0};
    _fr = {Config::FR_IN1_PIN, Config::FR_IN2_PIN, Config::FR_PWM_PIN, 1};
    _rl = {Config::RL_IN1_PIN, Config::RL_IN2_PIN, Config::RL_PWM_PIN, 2};
    _rr = {Config::RR_IN1_PIN, Config::RR_IN2_PIN, Config::RR_PWM_PIN, 3};
}

void MotorDriver::begin() {
    // Set pin modes
    pinMode(_fl.in1, OUTPUT);
    pinMode(_fl.in2, OUTPUT);
    pinMode(_fr.in1, OUTPUT);
    pinMode(_fr.in2, OUTPUT);
    pinMode(_rl.in1, OUTPUT);
    pinMode(_rl.in2, OUTPUT);
    pinMode(_rr.in1, OUTPUT);
    pinMode(_rr.in2, OUTPUT);
    // Initialize PWM channels
    uint32_t freq = 20000;    // 20 kHz PWM frequency
    uint8_t resolution = 8;    // 8-bit resolution
    ledcSetup(_fl.pwmChannel, freq, resolution);
    ledcAttachPin(_fl.pwm, _fl.pwmChannel);
    ledcSetup(_fr.pwmChannel, freq, resolution);
    ledcAttachPin(_fr.pwm, _fr.pwmChannel);
    ledcSetup(_rl.pwmChannel, freq, resolution);
    ledcAttachPin(_rl.pwm, _rl.pwmChannel);
    ledcSetup(_rr.pwmChannel, freq, resolution);
    ledcAttachPin(_rr.pwm, _rr.pwmChannel);
    // Initially stop all motors
    stop();
}

void MotorDriver::setWheelSpeed(const WheelSpeed &speed) {
    setSingleMotor(_fl, speed.fl);
    setSingleMotor(_fr, speed.fr);
    setSingleMotor(_rl, speed.rl);
    setSingleMotor(_rr, speed.rr);
}

void MotorDriver::stop() {
    // Set all motors to zero speed
    setWheelSpeed({0.0f, 0.0f, 0.0f, 0.0f});
}

void MotorDriver::setSingleMotor(const MotorConfig &mc, float radPerSec) {
    // Convert speed to PWM duty cycle
    float speedAbs = fabsf(radPerSec);
    // Constrain to maximum wheel speed
    if (speedAbs > Config::MAX_WHEEL_SPEED_RAD_S) {
        speedAbs = Config::MAX_WHEEL_SPEED_RAD_S;
    }
    uint16_t pwmVal = (uint16_t)(speedAbs / Config::MAX_WHEEL_SPEED_RAD_S * Config::MAX_PWM);
    // Direction control
    if (radPerSec > 0.0f) {
        // Forward: IN1 HIGH, IN2 LOW
        digitalWrite(mc.in1, HIGH);
        digitalWrite(mc.in2, LOW);
        ledcWrite(mc.pwmChannel, pwmVal);
    } else if (radPerSec < 0.0f) {
        // Reverse: IN1 LOW, IN2 HIGH
        digitalWrite(mc.in1, LOW);
        digitalWrite(mc.in2, HIGH);
        ledcWrite(mc.pwmChannel, pwmVal);
    } else {
        // Stop: both direction pins LOW, PWM 0
        digitalWrite(mc.in1, LOW);
        digitalWrite(mc.in2, LOW);
        ledcWrite(mc.pwmChannel, 0);
    }
}