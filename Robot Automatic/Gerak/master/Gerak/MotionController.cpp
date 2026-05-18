// MotionController.cpp
// implementasi

#include "MotionController.h"
#include <math.h>

MotionController::MotionController()
    : _kpPos(1.0f), _kpYaw(2.0f) {}

void MotionController::setGains(float kpPos, float kpYaw) {
    _kpPos = kpPos;
    _kpYaw = kpYaw;
}

VelocityCommand MotionController::computeCommand(const RobotState &state, const Waypoint &target) {
    VelocityCommand cmd{};
    // Compute error in world coordinates
    float ex_world = target.x - state.x;
    float ey_world = target.y - state.y;
    // Rotate error into body frame
    float cosYaw = cos(state.yaw);
    float sinYaw = sin(state.yaw);
    float ex_body =  cosYaw * ex_world + sinYaw * ey_world;
    float ey_body = -sinYaw * ex_world + cosYaw * ey_world;
    // Proportional control
    cmd.vx = _kpPos * ex_body;
    cmd.vy = _kpPos * ey_body;
    if (target.useYaw) {
        float yawError = target.yaw - state.yaw;
        while (yawError > M_PI) yawError -= 2 * M_PI;
        while (yawError < -M_PI) yawError += 2 * M_PI;
        cmd.wz = _kpYaw * yawError;
    } else {
        cmd.wz = 0.0f;
    }
    // Apply deadband: if error posisi kecil, vx dan vy = 0
    float posErrorMag = sqrtf(ex_world * ex_world + ey_world * ey_world);
    if (posErrorMag < Config::POSITION_TOLERANCE_M) {
        cmd.vx = 0.0f;
        cmd.vy = 0.0f;
    }
    if (target.useYaw && fabsf(cmd.wz) < Config::YAW_TOLERANCE_RAD) {
        cmd.wz = 0.0f;
    }
    // Limit outputs
    cmd.vx = constrainValue(cmd.vx, MAX_VX);
    cmd.vy = constrainValue(cmd.vy, MAX_VY);
    cmd.wz = constrainValue(cmd.wz, MAX_WZ);
    return cmd;
}

float MotionController::constrainValue(float val, float limit) {
    if (val > limit) return limit;
    if (val < -limit) return -limit;
    return val;
}