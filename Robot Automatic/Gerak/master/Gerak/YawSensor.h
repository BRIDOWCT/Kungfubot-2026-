// YawSensor.h
// Abstracts yaw measurement. Initially provides a dummy mode for testing
// without hardware. Later integration with GY-91/MPU and magnetometer
// can be added by replacing the dummy implementation in update().

#pragma once

#include <Arduino.h>

class YawSensor {
public:
    YawSensor();

    // Initialize the sensor. Returns true if initialization succeeds.
    bool begin();

    // Update the sensor reading. Should be called periodically.
    void update();

    // Retrieve the current yaw in radians. Range typically -pi to pi.
    float getYawRad() const;

    // Retrieve the current yaw in degrees.
    float getYawDeg() const;

    // Set a dummy yaw value in radians for testing when dummy mode
    // is enabled. This value will be returned by getYawRad().
    void setDummyYawRad(float yaw);

    // Enable or disable dummy mode. When true, sensor readings are
    // taken from the manually set dummy yaw. When false, the
    // implementation should read from the actual sensor.
    void enableDummyMode(bool enable);

private:
    bool  _dummyMode;
    float _dummyYaw;
    float _currentYaw;

    // Placeholder for sensor instance if using a real IMU/magnetometer
};