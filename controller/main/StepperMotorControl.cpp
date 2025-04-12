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

bool StepperMotorControl::isAtTarget() {
    return _stepper.distanceToGo() == 0;
}

void StepperMotorControl::home() {
//    Serial.println("Starting homing for motor");
    while (digitalRead(_limitSwitchPin) == 0) {
        // Only print every few iterations to avoid spamming
        static unsigned long lastPrint = 0;
        if (millis() - lastPrint > 500) {
//            Serial.print("Limit switch state: ");
//            Serial.println(digitalRead(_limitSwitchPin));
            lastPrint = millis();
        }
        _stepper.moveTo(_homePosition);
        _stepper.run();
    }
//    Serial.println("Homing complete");
    _stepper.setCurrentPosition(0);
    _stepper.stop();
}


void StepperMotorControl::stop() {
    _stepper.stop();
}
