import RPi.GPIO as GPIO
import spidev
import time

# Set GPIO mode
GPIO.setmode(GPIO.BCM)

# Define GPIO pins for the ADC0834 and joystick button
CS_PIN = 17   # Chip Select
DI_PIN = 27   # Data In/Out (MOSI)
DO_PIN = 27   # Data In/Out (MISO)
CLK_PIN = 18  # Clock (SPI SCK)
BUTTON_PIN = 16  # Joystick button pin

# Setup GPIO pins
GPIO.setup(CS_PIN, GPIO.OUT)
GPIO.setup(DI_PIN, GPIO.OUT)
GPIO.setup(DO_PIN, GPIO.IN)
GPIO.setup(CLK_PIN, GPIO.OUT)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Initialize SPI
spi = spidev.SpiDev()
spi.open(0, 0)  # Bus 0, Device 0 (chip select 0)
spi.max_speed_hz = 100000  # SPI speed (adjustable)
spi.mode = 0b00  # SPI mode 0 (CPOL=0, CPHA=0)

# Function to read from ADC0834
def read_adc(channel):
    print("Channel: ", channel)
    # ADC0834 command byte for single-ended read
    if channel == 0:
        command = 0x86  # Command to read channel 0 (VRX)
    elif channel == 1:
        command = 0x96  # Command to read channel 1 (VRY)
    else:
        raise ValueError("Invalid channel. Choose 0 or 1.")
    
    # Pull CS low to start communication
    GPIO.output(CS_PIN, GPIO.LOW)

    # Send command byte
    spi.xfer([command])

    # Read the response from the ADC (2 bytes)
    result = spi.xfer([0x00, 0x00])  # Send dummy data to receive the result

    # Pull CS high to end communication
    GPIO.output(CS_PIN, GPIO.HIGH)

    # Combine the two result bytes into one 8-bit value
    adc_value = result[1]  # The actual ADC value is in the second byte

    return adc_value

# Function to check if the joystick button is pressed
def button_pressed():
    return GPIO.input(BUTTON_PIN) == GPIO.LOW

# Main loop
try:
    while True:
        # Read the joystick axes (VRX = CH0, VRY = CH1)
        vrx_value = read_adc(0)  # Read X-axis value (CH0)
        vry_value = read_adc(1)  # Read Y-axis value (CH1)
        
        # Print joystick values
        print(f"Joystick X: {vrx_value} Y: {vry_value}")
        
        # Check if button is pressed
        if button_pressed():
            print("Button pressed!")
        
        # Delay for a short time before the next reading
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Program terminated")

finally:
    GPIO.cleanup()  # Clean up GPIO settings
