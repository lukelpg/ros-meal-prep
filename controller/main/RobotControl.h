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

private:
    MotorControl* _motorX;
    MotorControl* _motorY;
    MotorControl* _motorZ;
    bool _home;  // Flag to check if the robot is homed
};

#endif // ROBOTCONTROL_H
