import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import time
import Adafruit_ADS1x15

# Create an ADS1115 instance
ADC = Adafruit_ADS1x15.ADS1115()

# Set the gain to ±4.096V (adjust if needed)
GAIN = 1

# Specify the ADC channels for the joystick axes
X_CHANNEL = 0
Y_CHANNEL = 1

# Read center values initially to calculate thresholds
CENTER_SAMPLES = 100
x_center = sum([ADC.read_adc(X_CHANNEL, gain=GAIN) for _ in range(CENTER_SAMPLES)]) / CENTER_SAMPLES
y_center = sum([ADC.read_adc(Y_CHANNEL, gain=GAIN) for _ in range(CENTER_SAMPLES)]) / CENTER_SAMPLES

# Calculate thresholds based on center values
THRESHOLD_DELTA = 200  # Adjust this value as needed
LEFT_THRESHOLD = x_center - THRESHOLD_DELTA
RIGHT_THRESHOLD = x_center + THRESHOLD_DELTA
UP_THRESHOLD = y_center - THRESHOLD_DELTA
DOWN_THRESHOLD = y_center + THRESHOLD_DELTA

# Command constants
COMMAND_NO = 0x00
COMMAND_LEFT = 0x01
COMMAND_RIGHT = 0x02
COMMAND_UP = 0x04
COMMAND_DOWN = 0x08

class JoystickNode(Node):
    def __init__(self):
        super().__init__('joystick_node')
        self.joy_pub = self.create_publisher(Joy, 'joy', 10)
        self.timer = self.create_timer(0.1, self.publish_joystick_data)  # Periodically call publish_joystick_data

    def publish_joystick_data(self):
        # Read joystick values
        value_X = ADC.read_adc(X_CHANNEL, gain=GAIN)
        value_Y = ADC.read_adc(Y_CHANNEL, gain=GAIN)

        command = COMMAND_NO

        # Check joystick direction based on X and Y values
        if value_X < LEFT_THRESHOLD:
            command |= COMMAND_LEFT
        elif value_X > RIGHT_THRESHOLD:
            command |= COMMAND_RIGHT

        if value_Y < UP_THRESHOLD:
            command |= COMMAND_UP
        elif value_Y > DOWN_THRESHOLD:
            command |= COMMAND_DOWN

        # Create Joy message to publish
        joy_msg = Joy()

        # Assign joystick button states
        joy_msg.buttons = [1 if command & COMMAND_LEFT else 0,
                           1 if command & COMMAND_RIGHT else 0,
                           1 if command & COMMAND_UP else 0,
                           1 if command & COMMAND_DOWN else 0]

        # Map the joystick axes values to a range of -1 to 1
        joy_msg.axes = [
            (value_X - x_center) / 32767.0,  # Normalize X-axis value
            (value_Y - y_center) / 32767.0   # Normalize Y-axis value
        ]

        # Publish the joystick message
        self.joy_pub.publish(joy_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JoystickNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
