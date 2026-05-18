// Planner.h
//implementasi  waypoint-based planner. planner buat urutan waypoint/arah

#pragma once

#include <Arduino.h>
#include "Types.h"
#include "Config.h"

class Planner {
public:
    Planner();

    // Set a list of waypoints and count
    void setWaypoints(const Waypoint *points, int count);

    // Set a single target (overrides any previous list)
    void setSingleTarget(const Waypoint &target);

    // Call periodically with the current robot state. Will update the
    // internal target index when waypoints are reached.
    void update(const RobotState &state);

    // Get the current target waypoint
    Waypoint getCurrentTarget() const;

    // Return true if all waypoints have been reached
    bool isFinished() const;

    // Position error relative to current target in world frame
    float getPositionErrorX() const;
    float getPositionErrorY() const;
    float getYawError() const;

    // Return true if current target is reached
    bool isCurrentCheckpointReached() const;

private:
    const Waypoint *_points;
    int _count;
    int _currentIndex;
    float _positionTol;
    float _yawTol;

    // Store the most recent position error (world frame) and yaw error
    float _errorX;
    float _errorY;
    float _errorYaw;
};