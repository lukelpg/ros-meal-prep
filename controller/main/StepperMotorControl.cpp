#include <Arduino.h> 
#include "StepperMotorControl.h"

StepperMotorControl::StepperMotorControl(int stepPin, int dirPin, int enablePin, int limitSwitchPin)
    : _stepPin(stepPin), _dirPin(dirPin), _enablePin(enablePin), _limitSwitchPin(limitSwitchPin), _stepper(AccelStepper::DRIVER, _stepPin, _dirPin) {}

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
    Serial.println("In homing function");
    // Homing function: move towards the limit switch
    while (digitalRead(_limitSwitchPin) == HIGH) {  // Check if limit switch is triggered (assuming HIGH means not pressed)
        Serial.println("In while of homing function");
        _stepper.moveTo(-10000);  // Move towards the switch
        _stepper.run();
    }
    _stepper.setCurrentPosition(0);  // Set position to 0 when the limit switch is hit
    _stepper.stop();  // Stop motor after homing
}

void StepperMotorControl::stop() {
    _stepper.stop();
}
