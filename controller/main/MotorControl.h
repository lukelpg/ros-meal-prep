#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

class MotorControl {
public:
    virtual void setup() = 0;  // Setup motor
    virtual void moveTo(int position) = 0;  // Move to specified position
    virtual void run() = 0;  // Update motor state
    virtual void home() = 0;  // Homing functionality
    virtual void stop() = 0;  // Stop the motor
    virtual bool isAtTarget() = 0;
    virtual ~MotorControl() {}
};

#endif // MOTORCONTROL_H
