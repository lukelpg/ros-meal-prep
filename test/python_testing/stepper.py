import RPi.GPIO as GPIO
import time

# Set up GPIO mode
GPIO.setmode(GPIO.BCM)

# Define the GPIO pins for L293D
IN1 = 4   # Pin 2 on L293D
IN2 = 17  # Pin 3 on L293D
IN3 = 23  # Pin 9 on L293D
IN4 = 24  # Pin 10 on L293D

# Set the GPIO pins as output
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)

# Function to rotate the stepper motor
def stepper_rotate(steps, delay=0.01):
    # Define the step sequence for a 4-step stepper motor (full-step)
    sequence = [
        [GPIO.HIGH, GPIO.LOW, GPIO.LOW, GPIO.LOW],  # Step 1
        [GPIO.HIGH, GPIO.HIGH, GPIO.LOW, GPIO.LOW],  # Step 2
        [GPIO.LOW, GPIO.HIGH, GPIO.LOW, GPIO.LOW],  # Step 3
        [GPIO.LOW, GPIO.HIGH, GPIO.HIGH, GPIO.LOW],  # Step 4
        [GPIO.LOW, GPIO.LOW, GPIO.HIGH, GPIO.LOW],  # Step 5
        [GPIO.LOW, GPIO.LOW, GPIO.HIGH, GPIO.HIGH],  # Step 6
        [GPIO.LOW, GPIO.LOW, GPIO.LOW, GPIO.HIGH],  # Step 7
        [GPIO.HIGH, GPIO.LOW, GPIO.LOW, GPIO.HIGH]   # Step 8
    ]
    
    for _ in range(steps):
        for step in sequence:
            GPIO.output(IN1, step[0])
            GPIO.output(IN2, step[1])
            GPIO.output(IN3, step[2])
            GPIO.output(IN4, step[3])
            print("stepped")
            time.sleep(delay)

# Rotate the stepper motor 512 steps (one full rotation) with a delay of 0.01 seconds between steps
try:
    stepper_rotate(512, 0.01)  # Adjust the number of steps for your specific motor
finally:
    GPIO.cleanup()  # Clean up GPIO settings when the program is finished
