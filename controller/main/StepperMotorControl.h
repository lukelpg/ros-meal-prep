#ifndef STEPPERMOTORCONTROL_H
#define STEPPERMOTORCONTROL_H

#include "MotorControl.h"
#include <AccelStepper.h>

class StepperMotorControl : public MotorControl {
public:
    StepperMotorControl(int stepPin, int dirPin, int enablePin, int limitSwitchPin);
    
    void setup() override;
    void moveTo(int position) override;
    void run() override;
    void home() override;
    void stop() override;

private:
    int _stepPin, _dirPin, _enablePin, _limitSwitchPin;
    AccelStepper _stepper;
};

#endif // STEPPERMOTORCONTROL_H
