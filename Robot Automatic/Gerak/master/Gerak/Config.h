// Config.h
//parameter konfigurasu. buat atur nilai
#pragma once

#include <Arduino.h>

namespace Config {

// ===================== I2C Configuration =====================
// Define the I2C pins used on the ESP32 and bus frequency. The
// ESP32 allows any GPIO for I2C but common defaults are shown.
static const int I2C_SDA_PIN = 21;
static const int I2C_SCL_PIN = 22;
static const uint32_t I2C_FREQUENCY_HZ = 400000; // 400 kHz for fast mode

// PCA9548A multiplexer address (default 0x70)
static const uint8_t PCA9548A_ADDRESS = 0x70;

// channel multiplexernya
static const uint8_t PCA_CH_AS5600_X = 0;
static const uint8_t PCA_CH_AS5600_Y = 1;
static const uint8_t PCA_CH_GY91    = 2;

// ===================== AS5600 Encoder Configuration =====================
// AS5600 default I2C address and registers
static const uint8_t AS5600_ADDRESS      = 0x36;
static const uint8_t AS5600_REG_RAW_ANGLE = 0x0C; // Register to read raw angle

// Raw resolution (12-bit) counts per revolution
static const uint16_t AS5600_RESOLUTION = 4096;

// Calibration values: conversion factor from counts to meters for each
// incremental encoder wheel. These values should be measured for your
// odometry wheels during calibration.
static const float METER_PER_COUNT_X = 0.0005f; // Example: 0.5 mm per count
static const float METER_PER_COUNT_Y = 0.0005f;

// Invert direction flags: set to true if the encoder counts
// increase when the wheel moves in the negative body axis direction.
static const bool INVERT_DIRECTION_X = false;
static const bool INVERT_DIRECTION_Y = false;

// ===================== Sampling Configuration =====================
// Sampling frequencies (in Hz) for each sensor and control loops.
static const uint32_t ODOM_SAMPLE_HZ     = 200; // Odometry wheel sample rate
static const uint32_t YAW_SAMPLE_HZ      = 100; // Yaw sensor sample rate
static const uint32_t CONTROL_LOOP_HZ    = 50;  // Motion control loop rate
static const uint32_t DEBUG_PRINT_HZ     = 10;  // Serial debug print rate

// ===================== Odometry Configuration =====================
// Whether to apply wheel offset compensation for the odometry wheels.
static const bool USE_WHEEL_OFFSET_COMPENSATION = true;

// Position of the X-axis odometry wheel relative to the robot centre
// expressed in meters (body frame). Positive x = forward, positive y = left.
static const float WHEEL_X_OFFSET_X = 0.0f;
static const float WHEEL_X_OFFSET_Y = 0.05f; // e.g. 5 cm to the left of centre

// Position of the Y-axis odometry wheel relative to the robot centre
static const float WHEEL_Y_OFFSET_X = -0.05f; // e.g. 5 cm to the rear of centre
static const float WHEEL_Y_OFFSET_Y = 0.0f;

// ===================== Mecanum Kinematics Configuration =====================
// Dimensions of the robot for mecanum kinematics. Lx and Ly represent
// half of the wheelbase (distance from robot centre to wheel along x or y).
static const float WHEEL_RADIUS = 0.05f; // 5 cm wheel radius
static const float Lx = 0.15f;           // 15 cm half-length
static const float Ly = 0.12f;           // 12 cm half-width
// Derived kinematic constant K = Lx + Ly
static const float K = Lx + Ly;

// Maximum wheel speed in rad/s and maximum PWM value
static const float MAX_WHEEL_SPEED_RAD_S = 10.0f; // rad/s
static const uint16_t MAX_PWM            = 255;   // 8-bit PWM resolution

// ===================== Motor Pins Configuration =====================
// Define the pins for each of the four motors. These pins must be
// assigned according to how your motor driver is wired to the ESP32.
// Each motor has two direction pins (IN1, IN2) and one PWM pin.
// Front Left (FL)
static const int FL_IN1_PIN   = 13;
static const int FL_IN2_PIN   = 12;
static const int FL_PWM_PIN   = 14;

// Front Right (FR)
static const int FR_IN1_PIN   = 27;
static const int FR_IN2_PIN   = 26;
static const int FR_PWM_PIN   = 25;

// Rear Left (RL)
static const int RL_IN1_PIN   = 33;
static const int RL_IN2_PIN   = 32;
static const int RL_PWM_PIN   = 15;

// Rear Right (RR)
static const int RR_IN1_PIN   = 19;
static const int RR_IN2_PIN   = 18;
static const int RR_PWM_PIN   = 5;

// ===================== Planner Configuration =====================
// Tolerances for determining if a waypoint has been reached
static const float POSITION_TOLERANCE_M = 0.02f; // 2 cm tolerance
static const float YAW_TOLERANCE_RAD    = 0.05f; // ~3 degrees tolerance

// ===================== Debug/Test Mode =====================
// Select the operational mode of the firmware. Modes include:
// 0: SENSOR_TEST_MODE - read sensors only and print values
// 1: ODOM_TEST_MODE   - update odometry but do not drive motors
// 2: FULL_AUTO_MODE   - run planner and control motors
static const uint8_t RUN_MODE = 2;

} // namespace Config