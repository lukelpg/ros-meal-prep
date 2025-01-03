#include <Arduino.h> 
#include "RobotControl.h"

RobotControl::RobotControl(MotorControl* motorX, MotorControl* motorY, MotorControl* motorZ)
    : _motorX(motorX), _motorY(motorY), _motorZ(motorZ), _home(false) {}

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

void RobotControl::loop() {
    
    if (!_home) {
        Serial.println("Home motors called");
        homeMotors();  // Home the motors at the start
    }

    // Move the motors and call their run functions
    _motorX->run();
    _motorY->run();
    _motorZ->run();
}
