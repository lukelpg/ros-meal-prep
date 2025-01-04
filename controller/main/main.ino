#include "RobotControl.h"
#include "StepperMotorControl.h"

// Instantiate the motor controls with pins (Step pin, Dir pin, Enable pin, Limit switch pin)
StepperMotorControl motorX(2, 5, 8, 9, 20000);  // X motor
StepperMotorControl motorY(3, 6, 8, 10, 20000); // Y motor
StepperMotorControl motorZ(4, 7, 8, 11, -20000); // Z motor

// Create the RobotControl object
RobotControl robot(&motorX, &motorY, &motorZ);

void setup() {
    Serial.begin(9600);
    robot.setup();

    // Example square
    robot.addWaypoint(-200, -200, 0);
    robot.addWaypoint(-800, -200, 0);
    robot.addWaypoint(-800, -800, 0);  
    robot.addWaypoint(-200, -800, 0);
    robot.addWaypoint(-200, -200, 0);   
}

void loop() {
    robot.loop();  // This will continuously check and run the motors
}
