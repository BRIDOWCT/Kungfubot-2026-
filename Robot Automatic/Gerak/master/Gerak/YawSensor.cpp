// YawSensor.cpp
// implementasi
#include "YawSensor.h"
#include <math.h>

YawSensor::YawSensor()
    : _dummyMode(true), _dummyYaw(0.0f), _currentYaw(0.0f) {}

bool YawSensor::begin() {
    // In a real implementation, initialise the IMU/magnetometer here.
    // For example, using the Wire library to set up the GY-91.
    // Since this code is meant to compile without that dependency,
    // we simply return true.
    return true;
}

void YawSensor::update() {
    if (_dummyMode) {
        // In dummy mode, simply set the current yaw to the dummy value.
        _currentYaw = _dummyYaw;
    } else {
        // TODO: Replace this section with sensor reading code.
        // This could read gyroscope/accelerometer/magnetometer data from
        // a GY-91 or other IMU to compute yaw. For now, we set yaw to 0.
        _currentYaw = 0.0f;
    }
}

float YawSensor::getYawRad() const {
    return _currentYaw;
}

float YawSensor::getYawDeg() const {
    return _currentYaw * 180.0f / PI;
}

void YawSensor::setDummyYawRad(float yaw) {
    // Normalise yaw to -pi..pi for consistency
    while (yaw > PI) yaw -= 2 * PI;
    while (yaw < -PI) yaw += 2 * PI;
    _dummyYaw = yaw;
}

void YawSensor::enableDummyMode(bool enable) {
    _dummyMode = enable;
}