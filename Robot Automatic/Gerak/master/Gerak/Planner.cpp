// Planner.cpp
//implementasi

#include "Planner.h"
#include <math.h>

Planner::Planner()
    : _points(nullptr), _count(0), _currentIndex(0),
      _positionTol(Config::POSITION_TOLERANCE_M),
      _yawTol(Config::YAW_TOLERANCE_RAD),
      _errorX(0.0f), _errorY(0.0f), _errorYaw(0.0f) {}

void Planner::setWaypoints(const Waypoint *points, int count) {
    _points = points;
    _count = count;
    _currentIndex = 0;
}

void Planner::setSingleTarget(const Waypoint &target) {
    _points = &target;
    _count = 1;
    _currentIndex = 0;
}

void Planner::update(const RobotState &state) {
    if (isFinished()) {
        return;
    }
    const Waypoint &target = _points[_currentIndex];
    _errorX = target.x - state.x;
    _errorY = target.y - state.y;
    _errorYaw = target.yaw - state.yaw;
    // Normalise yaw error
    while (_errorYaw > M_PI) _errorYaw -= 2 * M_PI;
    while (_errorYaw < -M_PI) _errorYaw += 2 * M_PI;
    float posError = sqrtf(_errorX * _errorX + _errorY * _errorY);
    if (posError < _positionTol && (!target.useYaw || fabsf(_errorYaw) < _yawTol)) {
        // Reached current checkpoint
        _currentIndex++;
    }
}

Waypoint Planner::getCurrentTarget() const {
    if (isFinished()) {
        // If finished, return the last point as a dummy
        return _points[_count - 1];
    }
    return _points[_currentIndex];
}

bool Planner::isFinished() const {
    return (_currentIndex >= _count);
}

float Planner::getPositionErrorX() const {
    return _errorX;
}

float Planner::getPositionErrorY() const {
    return _errorY;
}

float Planner::getYawError() const {
    return _errorYaw;
}

bool Planner::isCurrentCheckpointReached() const {
    // Consider the checkpoint reached if planner would advance the index
    float posError = sqrtf(_errorX * _errorX + _errorY * _errorY);
    return posError < _positionTol && (fabsf(_errorYaw) < _yawTol || !_points[_currentIndex].useYaw);
}