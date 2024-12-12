#include <AccelStepper.h>

// Pin Definitions
#define ENABLE_PIN 8

#define MOTOR_X_STEP_PIN 2
#define MOTOR_X_DIR_PIN 5
#define LIMIT_SWITCH_PIN_X 9

#define MOTOR_Y_STEP_PIN 3
#define MOTOR_Y_DIR_PIN 6
#define LIMIT_SWITCH_PIN_Y 10

#define MOTOR_Z_STEP_PIN 4     // Step pin for Motor Z
#define MOTOR_Z_DIR_PIN 7      // Direction pin for Motor Z
#define LIMIT_SWITCH_PIN_Z 11

// Initialize the motors
AccelStepper motorX(1, MOTOR_X_STEP_PIN, MOTOR_X_DIR_PIN);
AccelStepper motorY(1, MOTOR_Y_STEP_PIN, MOTOR_Y_DIR_PIN);  // Motor Y
AccelStepper motorZ(1, MOTOR_Z_STEP_PIN, MOTOR_Z_DIR_PIN);  // Motor Z

int motorDirection = 0;
bool home = false;

void setup()
{
    Serial.begin(9600);  
   
   // Set motor enable and direction pins as output
   pinMode(ENABLE_PIN, OUTPUT);
   pinMode(LIMIT_SWITCH_PIN_Y, INPUT_PULLUP);  // Y- pin is configured as input with pull-up resistor
   pinMode(LIMIT_SWITCH_PIN_Z, INPUT_PULLUP);
   
   motorX.setEnablePin(ENABLE_PIN);
   motorY.setEnablePin(ENABLE_PIN);
   motorZ.setEnablePin(ENABLE_PIN);

   motorX.setPinsInverted(false, false, true);
   motorX.setAcceleration(1000);
   motorX.setMaxSpeed(500); // Set max speed for the motor
   motorX.enableOutputs();
   
   motorY.setPinsInverted(false, false, true);
   motorY.setAcceleration(1000);
   motorY.setMaxSpeed(500); // Set max speed for the motor
   motorY.enableOutputs();

   motorZ.setPinsInverted(false, false, true);
   motorZ.setAcceleration(1000);
   motorZ.setMaxSpeed(800); // Set max speed for the motor
   motorZ.enableOutputs();
   
   // Explicitly set the direction of the motor on startup (choose a direction)
   digitalWrite(MOTOR_X_DIR_PIN, LOW);
   digitalWrite(MOTOR_Y_DIR_PIN, LOW);  // Set the direction to LOW (change to HIGH if necessary)
   digitalWrite(MOTOR_Z_DIR_PIN, LOW);
}


void homeMotor() {

//  Serial.println(digitalRead(LIMIT_SWITCH_PIN_X));
  
  while (digitalRead(LIMIT_SWITCH_PIN_X) == 0) {
    Serial.println(digitalRead(LIMIT_SWITCH_PIN_X));
    motorX.moveTo(20000);
    motorX.run();
  }

  motorX.setCurrentPosition(0);             
  motorX.stop();
  
  while (digitalRead(LIMIT_SWITCH_PIN_Y) == 0) {
    motorY.moveTo(20000);
    motorY.run();
  }

  motorY.setCurrentPosition(0);             
  motorY.stop();

  while (digitalRead(LIMIT_SWITCH_PIN_Z) == 0) {
    motorZ.moveTo(-20000);
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
//    Serial.println(digitalRead(LIMIT_SWITCH_PIN_X));
    motorX.moveTo(-1000);
    motorX.run();

    motorY.moveTo(-1000);
    motorY.run();

    motorZ.moveTo(6000);
    motorZ.run();
  }

}
