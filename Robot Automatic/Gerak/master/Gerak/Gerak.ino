// kode utama
// program demonstrates sensor reading, odometry estimation, waypoint
// planning, motion control, and mecanum kinematics. It is designed
// for an ESP32 running in the Arduino environment.

#include <Arduino.h>
#include <Wire.h>
#include "Config.h"
#include "Types.h"
#include "PCA9548A_Manager.h"
#include "AS5600_Reader.h"
#include "IncrementalOdomSensor.h"
#include "YawSensor.h"
#include "OdometryEstimator.h"
#include "Planner.h"
#include "MotionController.h"
#include "MecanumKinematics.h"
#include "MotorDriver.h"

// Global objects
PCA9548A_Manager pca(Config::PCA9548A_ADDRESS);
AS5600_Reader encoderX(Config::AS5600_ADDRESS);
AS5600_Reader encoderY(Config::AS5600_ADDRESS);
IncrementalOdomSensor incrX(Config::METER_PER_COUNT_X, Config::INVERT_DIRECTION_X);
IncrementalOdomSensor incrY(Config::METER_PER_COUNT_Y, Config::INVERT_DIRECTION_Y);
YawSensor yawSensor;
OdometryEstimator odomEstimator;
Planner planner;
MotionController motionController;
MecanumKinematics mecanum;
MotorDriver motorDriver;

// Scheduler timing variables (microseconds)
unsigned long lastOdomSampleUs   = 0;
unsigned long lastYawSampleUs    = 0;
unsigned long lastControlUs      = 0;
unsigned long lastDebugPrintUs   = 0;

// Last measured yaw value
float currentYawRad = 0.0f;

// Waypoints for demonstration
static Waypoint waypoints[4];

void setup() {
    Serial.begin(115200);
    // Small delay to allow serial monitor to connect
    delay(100);
    Serial.println(F("RobotOdometryMecanum starting..."));
    // inisiasii sensor I2C
    Wire.begin(Config::I2C_SDA_PIN, Config::I2C_SCL_PIN, Config::I2C_FREQUENCY_HZ);
    // inisisaisi multiplexer
    if (!pca.begin()) {
        Serial.println(F("Failed to init PCA9548A"));
    }
    // inisiasi sensor
    encoderX.begin();
    encoderY.begin();
    yawSensor.begin(); //gy 91
    // motor driver
    motorDriver.begin();
    // Read initial raw values for encoders
    uint16_t rawX = 0;
    uint16_t rawY = 0;
    // Select channels and read
    if (pca.selectChannel(Config::PCA_CH_AS5600_X)) {
        if (!encoderX.readRawAngle(rawX)) {
            Serial.println(F("AS5600 X read failed during init"));
        }
    }
    if (pca.selectChannel(Config::PCA_CH_AS5600_Y)) {
        if (!encoderY.readRawAngle(rawY)) {
            Serial.println(F("AS5600 Y read failed during init"));
        }
    }
    // Initialise incremental sensors
    incrX.begin(rawX);
    incrY.begin(rawY);
    // Initialise odometry estimator with calibration
    odomEstimator.begin();
    // Optionally override calibration parameters here
    OdomCalibration calib;
    calib.meterPerCountX = Config::METER_PER_COUNT_X;
    calib.meterPerCountY = Config::METER_PER_COUNT_Y;
    calib.invertX = Config::INVERT_DIRECTION_X;
    calib.invertY = Config::INVERT_DIRECTION_Y;
    calib.useWheelOffsetCompensation = Config::USE_WHEEL_OFFSET_COMPENSATION;
    calib.wheelX_offsetX = Config::WHEEL_X_OFFSET_X;
    calib.wheelX_offsetY = Config::WHEEL_X_OFFSET_Y;
    calib.wheelY_offsetX = Config::WHEEL_Y_OFFSET_X;
    calib.wheelY_offsetY = Config::WHEEL_Y_OFFSET_Y;
    odomEstimator.setCalibration(calib);
    // Initialise yaw sensor (dummy by default)
    yawSensor.enableDummyMode(true); // start with dummy mode; disable if real sensor available
    currentYawRad = yawSensor.getYawRad();
    // Reset odometry to origin
    odomEstimator.reset(0.0f, 0.0f, currentYawRad);
    // Setup motion controller gains if needed
    motionController.setGains(1.0f, 2.0f);
    // Define waypoints
    waypoints[0] = {0.5f, 0.0f, 0.0f, false};
    waypoints[1] = {0.5f, 0.5f, 0.0f, false};
    waypoints[2] = {0.0f, 0.5f, 0.0f, false};
    waypoints[3] = {0.0f, 0.0f, 0.0f, false};
    // Assign waypoints to planner
    planner.setWaypoints(waypoints, 4);
    // Initialize scheduler timings
    unsigned long now = micros();
    lastOdomSampleUs  = now;
    lastYawSampleUs   = now;
    lastControlUs     = now;
    lastDebugPrintUs  = now;
}

void loop() {
    unsigned long now = micros();
    // Odometry sampling
    const unsigned long odomIntervalUs = 1000000UL / Config::ODOM_SAMPLE_HZ;
    if (now - lastOdomSampleUs >= odomIntervalUs) {
        lastOdomSampleUs = now;
        // Read X encoder
        uint16_t rawX;
        if (pca.selectChannel(Config::PCA_CH_AS5600_X) && encoderX.readRawAngle(rawX)) {
            incrX.update(rawX);
        } else {
            Serial.println(F("Warning: Failed reading AS5600 X"));
        }
        // Read Y encoder
        uint16_t rawY;
        if (pca.selectChannel(Config::PCA_CH_AS5600_Y) && encoderY.readRawAngle(rawY)) {
            incrY.update(rawY);
        } else {
            Serial.println(F("Warning: Failed reading AS5600 Y"));
        }
        // Update odometry estimator with last yaw
        float dx = incrX.getDeltaDistanceMeter();
        float dy = incrY.getDeltaDistanceMeter();
        odomEstimator.update(dx, dy, currentYawRad);
    }
    // Yaw sampling
    const unsigned long yawIntervalUs = 1000000UL / Config::YAW_SAMPLE_HZ;
    if (now - lastYawSampleUs >= yawIntervalUs) {
        lastYawSampleUs = now;
        // Select yaw sensor channel
        if (pca.selectChannel(Config::PCA_CH_GY91)) {
            yawSensor.update();
            currentYawRad = yawSensor.getYawRad();
        }
    }
    // Control loop
    const unsigned long controlIntervalUs = 1000000UL / Config::CONTROL_LOOP_HZ;
    if (now - lastControlUs >= controlIntervalUs) {
        lastControlUs = now;
        // Always update planner state with odometry
        RobotState state = odomEstimator.getState();
        // If not finished, compute command and drive motors
        if (Config::RUN_MODE == 2) {
            planner.update(state);
            if (!planner.isFinished()) {
                Waypoint target = planner.getCurrentTarget();
                VelocityCommand cmd = motionController.computeCommand(state, target);
                WheelSpeed ws = mecanum.inverse(cmd);
                ws = mecanum.normalizeWheelSpeed(ws);
                motorDriver.setWheelSpeed(ws);
            } else {
                motorDriver.stop();
            }
        } else if (Config::RUN_MODE == 1) {
            // ODOM_TEST_MODE: update planner but do not drive motors
            planner.update(state);
            motorDriver.stop();
        } else if (Config::RUN_MODE == 0) {
            // SENSOR_TEST_MODE: do nothing with odom or planner
            motorDriver.stop();
        }
    }
    // Debug printing
    const unsigned long debugIntervalUs = 1000000UL / Config::DEBUG_PRINT_HZ;
    if (now - lastDebugPrintUs >= debugIntervalUs) {
        lastDebugPrintUs = now;
        RobotState state = odomEstimator.getState();
        Serial.print(F("X: ")); Serial.print(state.x, 3);
        Serial.print(F(" m, Y: ")); Serial.print(state.y, 3);
        Serial.print(F(" m, Yaw: ")); Serial.print(state.yaw * 180.0f / PI, 1);
        Serial.print(F(" deg, dX: ")); Serial.print(incrX.getDeltaDistanceMeter(), 4);
        Serial.print(F(" m, dY: ")); Serial.print(incrY.getDeltaDistanceMeter(), 4);
        Serial.print(F(" m"));
        if (Config::RUN_MODE == 2) {
            if (!planner.isFinished()) {
                Waypoint tgt = planner.getCurrentTarget();
                Serial.print(F(", Target: (")); Serial.print(tgt.x, 2); Serial.print(F(",")); Serial.print(tgt.y, 2); Serial.print(F(")"));
                Serial.print(F(", ErrX: ")); Serial.print(planner.getPositionErrorX(), 3);
                Serial.print(F(", ErrY: ")); Serial.print(planner.getPositionErrorY(), 3);
                Serial.print(F(", ErrYaw: ")); Serial.print(planner.getYawError(), 2);
            } else {
                Serial.print(F(", Planner done"));
            }
        }
        Serial.println();
    }
}