import serial
import time

class SerialInterface:
    def __init__(self, port, baudrate=115200, timeout=1):
        """
        Initializes the serial interface to communicate with the microcontroller.
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None

    def connect(self):
        """
        Opens the serial port for communication.
        """
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            print(f"Connected to {self.port} at {self.baudrate} baud.")
        except serial.SerialException as e:
            print(f"Error opening serial port: {e}")
            raise e

    def disconnect(self):
        """
        Closes the serial connection.
        """
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Serial port closed.")

    def send_data(self, data):
        """
        Sends data over the serial interface.
        """
        if not self.ser or not self.ser.is_open:
            print("Serial port not open.")
            return
        
        self.ser.write(data.encode('utf-8'))  # Assume data is string
        print(f"Sent: {data}")

    def receive_data(self):
        """
        Receives data from the serial interface.
        """
        if not self.ser or not self.ser.is_open:
            print("Serial port not open.")
            return None
        
        data = self.ser.readline().decode('utf-8').strip()
        return data

    def is_open(self):
        return self.ser and self.ser.is_open
