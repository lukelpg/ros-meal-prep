import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from serial_comm_pkg.serial_interface import SerialInterface

class SerialCommNode(Node):
    def __init__(self):
        super().__init__('serial_comm')
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)

        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        self.serial_interface = SerialInterface(serial_port, baudrate)
        self.serial_interface.connect()

        # Create a publisher for the 'serial_in' topic
        self.publisher = self.create_publisher(String, 'serial_in', 10)

        # Create a subscriber to listen to outgoing messages
        self.subscription = self.create_subscription(
            String,
            'serial_out',
            self.handle_outgoing_message,
            10
        )
        self.subscription

        # Create a timer to check for incoming data periodically
        self.timer = self.create_timer(0.1, self.receive_serial_data)

    def handle_outgoing_message(self, msg):
        """Handles outgoing messages from ROS topic to serial."""
        self.get_logger().info(f"Sending: {msg.data}")
        self.serial_interface.send_data(msg.data)

    def receive_serial_data(self):
        """Checks for incoming serial data and publishes it to a ROS topic."""
        data = self.serial_interface.receive_data()
        if data:
            self.get_logger().info(f"Received: {data}")
            self.publish_received_data(data)

    def publish_received_data(self, data):
        """Publishes received data to a ROS topic."""
        msg = String()
        msg.data = data
        self.publisher.publish(msg)  # Use the initialized publisher here

    def on_shutdown(self):
        """Handles shutdown and closes serial connection."""
        self.serial_interface.disconnect()


def main(args=None):
    rclpy.init(args=args)
    node = SerialCommNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        rclpy.shutdown()
