// OdometryEstimator.h
// gabungan incremental odometri dengan yaw 
#pragma once

#include <Arduino.h>
#include "Types.h"

class OdometryEstimator {
public:
    OdometryEstimator();

    // Initialise internal state
    void begin();

    // Reset to a given pose
    void reset(float x, float y, float yaw);

    // Set calibration parameters for odometry
    void setCalibration(const OdomCalibration &calib);

    // Update odometry based on incremental sensor measurements (body frame)
    // dxSensorX: change measured along body x-axis odometry wheel (m)
    // dySensorY: change measured along body y-axis odometry wheel (m)
    // currentYawRad: current absolute yaw (rad)
    void update(float dxSensorX, float dySensorY, float currentYawRad);

    // Retrieve the current state estimate
    RobotState getState() const;

private:
    RobotState _state;
    OdomCalibration _calib;
    float _prevYaw;
    bool _isFirstUpdate;

    // Normalize an angle to [-pi, pi]
    static float normalizeAngleRad(float angle);
};