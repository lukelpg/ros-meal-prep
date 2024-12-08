#include <AccelStepper.h>

// Pin Definitions
#define MOTOR_Y_ENABLE_PIN 8
#define MOTOR_Y_STEP_PIN 3
#define MOTOR_Y_DIR_PIN 6
#define LIMIT_SWITCH_PIN 10  // Y- pin is typically 14 on the CNC shield (check your board's documentation)

// Initialize the motor
AccelStepper motorY(1, MOTOR_Y_STEP_PIN, MOTOR_Y_DIR_PIN);

int motorDirection = 0;

void setup()
{
   Serial.begin(9600);  
   
   // Set motor enable and direction pins as output
   pinMode(MOTOR_Y_ENABLE_PIN, OUTPUT);
   pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);  // Y- pin is configured as input with pull-up resistor

   motorY.setEnablePin(MOTOR_Y_ENABLE_PIN);
   motorY.setPinsInverted(false, false, true);
   motorY.setAcceleration(100);
   motorY.setMaxSpeed(500); // Set max speed for the motor
   motorY.enableOutputs();
   
   // Explicitly set the direction of the motor on startup (choose a direction)
   digitalWrite(MOTOR_Y_DIR_PIN, LOW);  // Set the direction to LOW (change to HIGH if necessary)
}


void loop()
{
   // Check if the limit switch is pressed (it will read LOW when pressed due to the pull-up resistor)
//   int switchState = digitalRead(LIMIT_SWITCH_PIN);  // Read the state of the limit switch
//   Serial.println(switchState); 
   
//   if (digitalRead(LIMIT_SWITCH_PIN) == LOW) {
//       // Reverse the direction of the motor
//       motorY.setCurrentPosition(0);  // Optionally reset the current position to 0
//       motorY.moveTo(-motorY.currentPosition()); // Move in the opposite direction
//       while (motorY.distanceToGo() != 0) {
//           motorY.run();  // Keep the motor running until it reaches the target position
//       }
//   }

   // Otherwise, move the motor normally

  if(digitalRead(LIMIT_SWITCH_PIN) == 1){
    motorDirection = 1;
  }

   

   if(motorDirection == 0){
    motorY.moveTo(7000);
   }else{
    motorY.moveTo(3000);
    }


    
   motorY.run();
}
