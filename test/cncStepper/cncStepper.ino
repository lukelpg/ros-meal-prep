#include <AccelStepper.h>

// Pin Definitions
#define MOTOR_Y_ENABLE_PIN 8
#define MOTOR_Y_STEP_PIN 3
#define MOTOR_Y_DIR_PIN 6
#define LIMIT_SWITCH_PIN 10  // Y- pin is typically 14 on the CNC shield (check your board's documentation)

// Initialize the motor
AccelStepper motorY(1, MOTOR_Y_STEP_PIN, MOTOR_Y_DIR_PIN);

int motorDirection = 0;
bool home = false;

void setup()
{
   Serial.begin(9600);  
   
   // Set motor enable and direction pins as output
   pinMode(MOTOR_Y_ENABLE_PIN, OUTPUT);
   pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);  // Y- pin is configured as input with pull-up resistor

   motorY.setEnablePin(MOTOR_Y_ENABLE_PIN);
   motorY.setPinsInverted(false, false, true);
   motorY.setAcceleration(1000);
   motorY.setMaxSpeed(750); // Set max speed for the motor
   motorY.enableOutputs();
   
   // Explicitly set the direction of the motor on startup (choose a direction)
   digitalWrite(MOTOR_Y_DIR_PIN, LOW);  // Set the direction to LOW (change to HIGH if necessary)
}


void homeMotor() {
  while (digitalRead(LIMIT_SWITCH_PIN) == 0) {
    motorY.moveTo(20000);
    motorY.run();
  }

  motorY.setCurrentPosition(0);             
  motorY.stop();
}


void loop()
{
  if (home == false){
    homeMotor();
    home = true;
  }else{
    motorY.moveTo(-6000);
    motorY.run();
  }

}
