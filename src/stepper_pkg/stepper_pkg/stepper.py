import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
import time

class SimpleNode(Node):
    def __init__(self):
        super().__init__('simple_node')
        self.get_logger().info('Hello, ROS 2!')

        # Setup GPIO pin 21 for output
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(21, GPIO.OUT)

        # Start a timer to blink the LED every second
        self.timer = self.create_timer(1.0, self.blink_led)

    def blink_led(self):
        # Blink the LED on and off
        GPIO.output(21, GPIO.HIGH)  # Turn LED on
        time.sleep(0.5)              # Wait for 0.5 seconds
        GPIO.output(21, GPIO.LOW)   # Turn LED off
        time.sleep(0.5)              # Wait for 0.5 seconds

    def destroy(self):
        # Clean up the GPIO settings when the node is destroyed
        GPIO.cleanup()

def main(args=None):
    rclpy.init(args=args)
    node = SimpleNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Clean up and shutdown
    node.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
