#include "RobotControl.h"
#include "StepperMotorControl.h"

// Instantiate the motor controls with pins (Step pin, Dir pin, Enable pin, Limit switch pin)
StepperMotorControl motorX(2, 5, 8, 9, 20000);  // X motor
StepperMotorControl motorY(3, 6, 8, 10, 20000); // Y motor
StepperMotorControl motorZ(4, 7, 8, 11, -20000); // Z motor

// Create the RobotControl object
RobotControl robot(&motorX, &motorY, &motorZ);

// Function to generate random waypoints within the defined bounds
void generateRandomWaypoints(int numWaypoints) {
    for (int i = 0; i < numWaypoints; i++) {
        // Generate random coordinates within the defined bounds
        int x = random(-1000, 1);  // x in range [0, -1000]
        int y = random(-1000, 1);  // y in range [0, -1000]
        int z = random(0, 6001);   // z in range [0, 6000]

        // Add the generated waypoint to the robot's path
        robot.addWaypoint(x, y, z);
        
        // Optionally print the waypoints to the serial monitor
        Serial.print("Waypoint ");
        Serial.print(i + 1);
        Serial.print(": x=");
        Serial.print(x);
        Serial.print(", y=");
        Serial.print(y);
        Serial.print(", z=");
        Serial.println(z);
    }
}

void setup() {
    Serial.begin(9600);
    robot.setup();

    // Generate and add random waypoints to the robot's path
//    generateRandomWaypoints(10);  // Generate 10 random waypoints

        // Example square
    robot.addWaypoint(-200, -200, 0);
    robot.addWaypoint(-800, -200, 0);
    robot.addWaypoint(-800, -800, 0);  
    robot.addWaypoint(-200, -800, 0);
    robot.addWaypoint(-200, -200, 0);  

    // You can also add specific waypoints if desired:
    // robot.addWaypoint(-200, -200, 0);   // Example waypoint 1
    // robot.addWaypoint(-800, -200, 0);   // Example waypoint 2
}

void loop() {
    robot.loop();  // This will continuously check and run the motors
}
