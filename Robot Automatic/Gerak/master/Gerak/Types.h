// Types.h
// struktur data 
#pragma once

#include <Arduino.h>

// posisi robot di dunia nyata
struct RobotState {
    float x;   // metres
    float y;   // metres
    float yaw; // radians
};

//  perubahann Incremental
struct BodyDelta {
    float dx;   // metres along +x body (forward)
    float dy;   // metres along +y body (left)
    float dyaw; // radians (CCW positive)
};

// perubahan Incremental (yaw removed)
struct WorldDelta {
    float dx; // metres in world frame
    float dy; // metres in world frame
};

// kecepatan
struct VelocityCommand {
    float vx; // m/s along +x body
    float vy; // m/s along +y body
    float wz; // rad/s CCW
};

// Wheel speeds in rad/s for each wheel
struct WheelSpeed {
    float fl; // front left
    float fr; // front right
    float rl; // rear left
    float rr; // rear right
};

// Waypoint/Checkpoint definition
struct Waypoint {
    float x;    // target x in world frame (m)
    float y;    // target y in world frame (m)
    float yaw;  // target heading (rad)
    bool useYaw; // whether yaw should be controlled
};

// kalibrasi parameter odometry  uuntuk sensore  incremental 
struct OdomCalibration {
    float meterPerCountX;
    float meterPerCountY;
    bool invertX;
    bool invertY;

    bool useWheelOffsetCompensation;

    float wheelX_offsetX;
    float wheelX_offsetY;

    float wheelY_offsetX;
    float wheelY_offsetY;
};