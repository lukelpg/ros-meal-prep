#include <Arduino.h> 
#include "StepperMotorControl.h"

StepperMotorControl::StepperMotorControl(int stepPin, int dirPin, int enablePin, int limitSwitchPin, int homePosition)
    : _stepPin(stepPin), _dirPin(dirPin), _enablePin(enablePin), _limitSwitchPin(limitSwitchPin), _homePosition(homePosition), _stepper(AccelStepper::DRIVER, _stepPin, _dirPin) {}

void StepperMotorControl::setup() {
    pinMode(_enablePin, OUTPUT);
    digitalWrite(_enablePin, LOW);  // Enable motor
    pinMode(_limitSwitchPin, INPUT_PULLUP);  // Limit switch as input with pull-up

    _stepper.setMaxSpeed(500);
    _stepper.setAcceleration(1000);
}

void StepperMotorControl::moveTo(int position) {
    _stepper.moveTo(position);
}

void StepperMotorControl::run() {
    _stepper.run();
}

void StepperMotorControl::home() {
//    Serial.println("In homing function");
    // Homing function: move towards the limit switch
    while (digitalRead(_limitSwitchPin) == 0) {
        Serial.println(digitalRead(_limitSwitchPin));
        _stepper.moveTo(_homePosition);  // Move towards the switch
        _stepper.run();
    }
    _stepper.setCurrentPosition(0);  // Set position to 0 when the limit switch is hit
    _stepper.stop();  // Stop motor after homing
}

void StepperMotorControl::stop() {
    _stepper.stop();
}
