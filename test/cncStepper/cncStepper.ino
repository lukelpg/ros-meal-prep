#include <AccelStepper.h>

// Pin Definitions
#define ENABLE_PIN 8

#define MOTOR_Y_STEP_PIN 3
#define MOTOR_Y_DIR_PIN 6
#define LIMIT_SWITCH_PIN_Y 10

#define MOTOR_Z_STEP_PIN 4     // Step pin for Motor Z
#define MOTOR_Z_DIR_PIN 7      // Direction pin for Motor Z
#define LIMIT_SWITCH_PIN_Z 11

// Initialize the motors
AccelStepper motorY(1, MOTOR_Y_STEP_PIN, MOTOR_Y_DIR_PIN);  // Motor Y
AccelStepper motorZ(1, MOTOR_Z_STEP_PIN, MOTOR_Z_DIR_PIN);  // Motor Z

int motorDirection = 0;
bool home = false;

void setup()
{
   // Serial.begin(9600);  
   
   // Set motor enable and direction pins as output
   pinMode(ENABLE_PIN, OUTPUT);
   pinMode(LIMIT_SWITCH_PIN_Y, INPUT_PULLUP);  // Y- pin is configured as input with pull-up resistor
   pinMode(LIMIT_SWITCH_PIN_Z, INPUT_PULLUP);
   

   motorY.setEnablePin(ENABLE_PIN);
   
   motorY.setPinsInverted(false, false, true);
   motorY.setAcceleration(1000);
   motorY.setMaxSpeed(750); // Set max speed for the motor
   motorY.enableOutputs();

   motorZ.setPinsInverted(false, false, true);
   motorZ.setAcceleration(1000);
   motorZ.setMaxSpeed(500); // Set max speed for the motor
   motorZ.enableOutputs();
   
   // Explicitly set the direction of the motor on startup (choose a direction)
   digitalWrite(MOTOR_Y_DIR_PIN, LOW);  // Set the direction to LOW (change to HIGH if necessary)
   digitalWrite(MOTOR_Z_DIR_PIN, LOW);
}


void homeMotor() {
//  while (digitalRead(LIMIT_SWITCH_PIN_Y) == 0) {
//    motorY.moveTo(20000);
//    motorY.run();
//  }
//
//  motorY.setCurrentPosition(0);             
//  motorY.stop();

  while (digitalRead(LIMIT_SWITCH_PIN_Z) == 0) {
    motorZ.moveTo(20000);
    motorZ.run();
  }
  
  motorZ.setCurrentPosition(0);             
  motorZ.stop();
}


void loop()
{
  if (home == false){
    homeMotor();
    home = true;
  }else{
    motorZ.moveTo(-3000);
    motorZ.run();

//    motorY.moveTo(-6000);
//    motorY.run();
  }

}
