#!/usr/bin/env python3
import serial
import time

def read_serial_data(ser):
    """Function to handle serial data reading with error handling."""
    try:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').rstrip()  # Ignore any invalid UTF-8 bytes
            return line
        return None
    except serial.SerialException as e:
        print(f"Serial error: {e}")
        return None
    except UnicodeDecodeError as e:
        print(f"Unicode decode error: {e}")
        return None
    except Exception as e:
        print(f"Unexpected error: {e}")
        return None

if __name__ == '__main__':
    try:
        ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  # Initialize serial port
        ser.reset_input_buffer()  # Reset input buffer
        print("Serial connection established successfully.")
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        exit(1)  # Exit if the serial port can't be opened

    while True:
        try:
            line = read_serial_data(ser)
            if line is not None:
                print(line)  # Print received line
            time.sleep(0.1)  # Small delay to avoid high CPU usage

        except KeyboardInterrupt:
            print("Program interrupted by user.")
            break
        except Exception as e:
            print(f"Unexpected error in main loop: {e}")
            time.sleep(1)  # Delay before retrying
