import serial
import time

# Set up the serial port (adjust 'COMx' or '/dev/ttyUSBx' depending on your system)
# For example, on Windows, you might use 'COM3', on Linux or macOS '/dev/ttyUSB0'.
ser = serial.Serial('/dev/ttyACM0', 115200)  # Replace with the correct port
time.sleep(2)  # Give some time for the connection to establish

while True:
    # Read data from ESP32 (blocking)
    if ser.in_waiting > 0:
        data = ser.readline().decode('utf-8').strip()
        print("Received from ESP32:", data)

    # Send data to ESP32
    ser.write(b"Hello from Python!\n")
    time.sleep(1)  # Send every 1 second
