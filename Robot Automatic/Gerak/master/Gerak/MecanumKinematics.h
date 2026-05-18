// MecanumKinematics.h
//  inverse kinematics, buat kecepatan sudut individual roda
#pragma once

#include <Arduino.h>
#include "Types.h"
#include "Config.h"

class MecanumKinematics {
public:
    MecanumKinematics();

    // Convert desired velocity command to individual wheel speeds (rad/s)
    WheelSpeed inverse(const VelocityCommand &cmd) const;

    // Normalize wheel speeds to ensure none exceed the maximum allowed
    WheelSpeed normalizeWheelSpeed(const WheelSpeed &ws) const;

private:
    float _radius;
    float _K;
    float _maxWheelSpeed;
};