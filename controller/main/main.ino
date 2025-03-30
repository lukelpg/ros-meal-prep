#include "RobotControl.h"
#include "StepperMotorControl.h"

// Instantiate the motor controls with pins (Step pin, Dir pin, Enable pin, Limit switch pin)
StepperMotorControl motorX(2, 5, 8, 9, 20000);  // X motor
StepperMotorControl motorY(3, 6, 8, 10, 20000); // Y motor
StepperMotorControl motorZ(4, 7, 8, 11, -20000); // Z motor

// Create the RobotControl object
RobotControl robot(&motorX, &motorY, &motorZ);

bool isMoving = false;

// This function encapsulates the logic to process a command string
void processCommand(String command) {
    if (command.startsWith("MOVE")) {
        // Parse the MOVE command and extract coordinates
        int firstComma = command.indexOf(',');
        int secondComma = command.indexOf(',', firstComma + 1);
        int thirdComma = command.indexOf(',', secondComma + 1);

        int x = command.substring(firstComma + 1, secondComma).toInt();
        int y = command.substring(secondComma + 1, thirdComma).toInt();
        int z = command.substring(thirdComma + 1).toInt();

        // Add the waypoint to the robot's path
        robot.addWaypoint(x, y, z);

        Serial.print("Waypoint added: ");
        Serial.print(x);
        Serial.print(", ");
        Serial.print(y);
        Serial.print(", ");
        Serial.println(z);
        Serial.println("ACK");
    }
    else if (command == "GO") {
        // Start the robot's movement
        isMoving = true;
        Serial.println("GO signal received. Robot will start moving.");
    }
}

// This function simulates sending a test square as a list of commands
void simulateTestSquare() {
    // Define the test square commands
    String testCommands[] = {
        "MOVE,-200,-200,0",
        "MOVE,-800,-200,0",
        "MOVE,-800,-800,0",
        "MOVE,-200,-800,0",
        "MOVE,-200,-200,0",
        "GO"
    };
    const int numCommands = sizeof(testCommands) / sizeof(testCommands[0]);

    // Loop through each test command and process it
    for (int i = 0; i < numCommands; i++) {
        processCommand(testCommands[i]);
        delay(100);  // Optional delay between commands
    }
}

void setup() {
    Serial.begin(115200);
    robot.setup();
    Serial.println("Robot setup complete");

    // Uncomment one of these options:
    // Option 1: Manually send commands via the Serial Monitor.
    // Option 2: Simulate the test square commands.
//    simulateTestSquare();
}

void loop() {
    // If data is available on the serial port, process it.
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        processCommand(command);
    }

    // If the robot is moving, call its loop method.
    if (isMoving) {
        Serial.println("Running robot loop");
        robot.loop();

        // Check if all waypoints have been processed
        if (robot.hasCompletedWaypoints()) {
            Serial.println("DONE");
            isMoving = false;
        }
    }
}
