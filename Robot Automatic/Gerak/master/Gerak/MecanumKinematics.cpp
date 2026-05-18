// MecanumKinematics.cpp
// implementasi

#include "MecanumKinematics.h"

MecanumKinematics::MecanumKinematics()
    : _radius(Config::WHEEL_RADIUS), _K(Config::K), _maxWheelSpeed(Config::MAX_WHEEL_SPEED_RAD_S) {}

WheelSpeed MecanumKinematics::inverse(const VelocityCommand &cmd) const {
    WheelSpeed ws{};
    // Compute each wheel speed using the standard mecanum IK equations
    ws.fl = (cmd.vx - cmd.vy - _K * cmd.wz) / _radius;
    ws.fr = (cmd.vx + cmd.vy + _K * cmd.wz) / _radius;
    ws.rl = (cmd.vx + cmd.vy - _K * cmd.wz) / _radius;
    ws.rr = (cmd.vx - cmd.vy + _K * cmd.wz) / _radius;
    return ws;
}

WheelSpeed MecanumKinematics::normalizeWheelSpeed(const WheelSpeed &ws) const {
    WheelSpeed scaled = ws;
    // Find the maximum absolute wheel speed
    float maxAbs = fabsf(ws.fl);
    maxAbs = max(maxAbs, fabsf(ws.fr));
    maxAbs = max(maxAbs, fabsf(ws.rl));
    maxAbs = max(maxAbs, fabsf(ws.rr));
    if (maxAbs > _maxWheelSpeed && maxAbs > 0.0f) {
        float scale = _maxWheelSpeed / maxAbs;
        scaled.fl *= scale;
        scaled.fr *= scale;
        scaled.rl *= scale;
        scaled.rr *= scale;
    }
    return scaled;
}