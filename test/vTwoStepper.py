import RPi.GPIO as GPIO
import time

# Define the GPIO pins connected to the stepper motor
pins = [4, 17, 23, 24]

# Set the GPIO mode
GPIO.setmode(GPIO.BCM)

# Set the GPIO pins as outputs
for pin in pins:
    GPIO.setup(pin, GPIO.OUT)

# Define the stepper motor sequence
# This is a common 4-step sequence for a stepper motor
step_sequence = [
    [1, 0, 0, 1],  # Step 1
    [1, 1, 0, 0],  # Step 2
    [0, 1, 1, 0],  # Step 3
    [0, 0, 1, 1]   # Step 4
]

# Function to perform the steps
def step_motor(steps, delay=0.01):
    for i in range(steps):
        for step in step_sequence:
            for pin, val in zip(pins, step):
                GPIO.output(pin, val)  # Set each pin according to the step
            time.sleep(delay)  # Delay between steps to control speed

# Main loop to spin the motor
try:
    while True:
        # Spin the motor in one direction
        step_motor(512)  # Adjust the number of steps for a full revolution
        
        # Optional: Add a delay between forward and backward motion
        time.sleep(1)  # Pause for 1 second
        
        # Spin the motor in the opposite direction
        step_motor(512)  # Adjust steps for reverse direction
        time.sleep(1)
        
except KeyboardInterrupt:
    print("Program interrupted")

finally:
    # Cleanup GPIO settings
    GPIO.cleanup()
