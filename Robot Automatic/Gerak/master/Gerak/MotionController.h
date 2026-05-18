// MotionController.h
// atur kecepatan dari posisi dan error dari target. proporsional

#pragma once

#include <Arduino.h>
#include "Types.h"
#include "Config.h"

class MotionController {
public:
    MotionController();

    // Compute the velocity command in body frame to move towards target
    VelocityCommand computeCommand(const RobotState &state, const Waypoint &target);

    // Set proportional gains for position and yaw
    void setGains(float kpPos, float kpYaw);

private:
    float _kpPos;
    float _kpYaw;

    // Maximum velocities (m/s and rad/s)
    static constexpr float MAX_VX = 0.5f;
    static constexpr float MAX_VY = 0.5f;
    static constexpr float MAX_WZ = 1.0f;

    // Helper to constrain a value within ±limit
    static float constrainValue(float val, float limit);
};