#ifndef ROBOTCONTROL_H
#define ROBOTCONTROL_H

#include "MotorControl.h"

class RobotControl {
public:
    RobotControl(MotorControl* motorX, MotorControl* motorY, MotorControl* motorZ);

    void setup();  // Initialize motors
    void homeMotors();  // Home all motors
    void moveTo(int x, int y, int z);  // Move to specific coordinates
    void loop();  // Main loop to control the robot

    // Path following
    void addWaypoint(int x, int y, int z);  // Add a waypoint to the path
    void moveToNextWaypoint();  // Move to the next waypoint

private:
    MotorControl* _motorX;
    MotorControl* _motorY;
    MotorControl* _motorZ;
    bool _home;  // Flag to check if the robot is homed

    // Path following variables
    static const int MAX_WAYPOINTS = 10;  // Maximum number of waypoints
    int _waypoints[MAX_WAYPOINTS][3];  // Array of waypoints (x, y, z)
    size_t _currentWaypointIndex;  // Index of the current waypoint
    size_t _waypointCount;  // Number of waypoints in the path
};

#endif // ROBOTCONTROL_H
