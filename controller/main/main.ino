#include "RobotControl.h"
#include "StepperMotorControl.h"

// Instantiate the motor controls with pins (Step pin, Dir pin, Enable pin, Limit switch pin)
StepperMotorControl motorX(2, 5, 8, 9, 20000);  // X motor
StepperMotorControl motorY(3, 6, 8, 10, 20000); // Y motor
StepperMotorControl motorZ(4, 7, 8, 11, -20000); // Z motor

// Create the RobotControl object
RobotControl robot(&motorX, &motorY, &motorZ);

bool isMoving = false;

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
    // generateRandomWaypoints(10);  // Generate 10 random waypoints

    // Example square
    // robot.addWaypoint(-200, -200, 0);
    // robot.addWaypoint(-800, -200, 0);
    // robot.addWaypoint(-800, -800, 0);  
    // robot.addWaypoint(-200, -800, 0);
    // robot.addWaypoint(-200, -200, 0);  
}

void loop() {
    // Check if there is incoming serial data
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');  // Read the incoming command

        if (command.startsWith("MOVE")) {
            // Parse the MOVE command and extract coordinates
            int x = command.substring(5, command.indexOf(',')).toInt();
            int y = command.substring(command.indexOf(',') + 1, command.lastIndexOf(',')).toInt();
            int z = command.substring(command.lastIndexOf(',') + 1).toInt();

            // Add the waypoint to the robot's path
            robot.addWaypoint(x, y, z);
            Serial.print("Waypoint added: ");
            Serial.print(x);
            Serial.print(", ");
            Serial.print(y);
            Serial.print(", ");
            Serial.println(z);
        }
        else if (command == "GO") {
            // When the GO signal is received, start the robot's movement
            isMoving = true;
            Serial.println("GO signal received. Robot will start moving.");
        }
    }

    // If the robot is in moving mode and there are waypoints in the queue
    if (isMoving) {
        robot.loop();  // This will process the movement

        // TODO: Something like this. Check if the robot has completed all the waypoints (you may need to track this in your code)
        // if (robot.hasCompletedWaypoints()) {  // Assuming the robot has a function that tracks if it completed all waypoints
        //     Serial.println("DONE");  // Send the DONE message back to the Pi
        //     isMoving = false;  // Stop the robot movement
        // }
    }
}
