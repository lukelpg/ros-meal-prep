#include <Arduino.h>
#include "RobotControl.h"

RobotControl::RobotControl(MotorControl* motorX, MotorControl* motorY, MotorControl* motorZ)
    : _motorX(motorX), _motorY(motorY), _motorZ(motorZ), _home(false), _currentWaypointIndex(0), _waypointCount(0) {}

void RobotControl::setup() {
    _home = false;
    _motorX->setup();
    _motorY->setup();
    _motorZ->setup();
}

void RobotControl::homeMotors() {
    if (!_home) {
        _motorX->home();
        _motorY->home();
        _motorZ->home();
        _home = true;
    }
}

void RobotControl::moveTo(int x, int y, int z) {
    _motorX->moveTo(x);
    _motorY->moveTo(y);
    _motorZ->moveTo(z);
}

void RobotControl::addWaypoint(int x, int y, int z) {
    if (_waypointCount < MAX_WAYPOINTS) {
        _waypoints[_waypointCount][0] = x;
        _waypoints[_waypointCount][1] = y;
        _waypoints[_waypointCount][2] = z;
        _waypointCount++;
    }
}

void RobotControl::moveToNextWaypoint() {
    if (_currentWaypointIndex < _waypointCount) {
        moveTo(_waypoints[_currentWaypointIndex][0], _waypoints[_currentWaypointIndex][1], _waypoints[_currentWaypointIndex][2]);
    }
}

void RobotControl::loop() {
    if (!_home) {
        homeMotors();  // Home the motors at the start
    }

    if (_currentWaypointIndex < _waypointCount) {
        // Move to the next waypoint
        moveToNextWaypoint();

        // Check if all motors have reached the target positions
        if (_motorX->isAtTarget() && _motorY->isAtTarget() && _motorZ->isAtTarget()) {
            _currentWaypointIndex++;  // Move to the next waypoint
        }
    }

    // Move the motors and call their run functions
    _motorX->run();
    _motorY->run();
    _motorZ->run();
}
