#ifndef STEPPERMOTORCONTROL_H
#define STEPPERMOTORCONTROL_H

#include "MotorControl.h"
#include <AccelStepper.h>

class StepperMotorControl : public MotorControl {
public:
    StepperMotorControl(int stepPin, int dirPin, int enablePin, int limitSwitchPin, int homePosition);
    
    void setup() override; 
    void moveTo(int position) override;
    void run() override;
    void home() override;
    void stop() override; 
    
    bool isAtTarget();
    
private:
    int _stepPin, _dirPin, _enablePin, _limitSwitchPin, _homePosition;
    AccelStepper _stepper;
};

#endif // STEPPERMOTORCONTROL_H
