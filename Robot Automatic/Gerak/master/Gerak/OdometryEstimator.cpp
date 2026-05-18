// OdometryEstimator.cpp
// implementasi
#include "OdometryEstimator.h"
#include "Config.h"
#include <math.h>

OdometryEstimator::OdometryEstimator()
    : _prevYaw(0.0f), _isFirstUpdate(true) {
    _state.x = 0.0f;
    _state.y = 0.0f;
    _state.yaw = 0.0f;
    // Default calibration
    _calib.meterPerCountX = Config::METER_PER_COUNT_X;
    _calib.meterPerCountY = Config::METER_PER_COUNT_Y;
    _calib.invertX = Config::INVERT_DIRECTION_X;
    _calib.invertY = Config::INVERT_DIRECTION_Y;
    _calib.useWheelOffsetCompensation = Config::USE_WHEEL_OFFSET_COMPENSATION;
    _calib.wheelX_offsetX = Config::WHEEL_X_OFFSET_X;
    _calib.wheelX_offsetY = Config::WHEEL_X_OFFSET_Y;
    _calib.wheelY_offsetX = Config::WHEEL_Y_OFFSET_X;
    _calib.wheelY_offsetY = Config::WHEEL_Y_OFFSET_Y;
}

void OdometryEstimator::begin() {
    // Nothing additional to initialize
}

void OdometryEstimator::reset(float x, float y, float yaw) {
    _state.x = x;
    _state.y = y;
    _state.yaw = yaw;
    _prevYaw = yaw;
    _isFirstUpdate = true;
}

void OdometryEstimator::setCalibration(const OdomCalibration &calib) {
    _calib = calib;
}

void OdometryEstimator::update(float dxSensorX, float dySensorY, float currentYawRad) {
    // Compute change in yaw since last update
    float dyaw = currentYawRad - _prevYaw;
    dyaw = normalizeAngleRad(dyaw);
    _prevYaw = currentYawRad;

    // Apply wheel offset compensation if enabled
    float dx_body, dy_body;
    if (_calib.useWheelOffsetCompensation) {
        // Effects of rotation on the sensor readings
        // For sensor X, offset relative to centre (rx, ry): dx_correction = -ry * dyaw
        float dx_rot = -_calib.wheelX_offsetY * dyaw;
        // For sensor Y: dy_correction = rx * dyaw
        float dy_rot = _calib.wheelY_offsetX * dyaw;
        dx_body = dxSensorX - dx_rot;
        dy_body = dySensorY - dy_rot;
    } else {
        dx_body = dxSensorX;
        dy_body = dySensorY;
    }

    // Transform to world frame using current yaw
    float cy = cos(currentYawRad);
    float sy = sin(currentYawRad);
    float dx_world = dx_body * cy - dy_body * sy;
    float dy_world = dx_body * sy + dy_body * cy;

    // Update pose
    if (_isFirstUpdate) {
        // On first update, avoid applying a large dyaw from initial difference
        _isFirstUpdate = false;
    }
    _state.x += dx_world;
    _state.y += dy_world;
    _state.yaw = currentYawRad;
}

RobotState OdometryEstimator::getState() const {
    return _state;
}

float OdometryEstimator::normalizeAngleRad(float angle) {
    while (angle > M_PI) angle -= 2.0f * (float)M_PI;
    while (angle < -M_PI) angle += 2.0f * (float)M_PI;
    return angle;
}