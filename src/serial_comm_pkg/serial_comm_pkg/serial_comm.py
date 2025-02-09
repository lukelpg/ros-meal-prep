import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from serial_comm_pkg.serial_interface import SerialInterface

class SerialCommNode(Node):
    def __init__(self):
        super().__init__('serial_comm')
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)

        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        self.serial_interface = SerialInterface(serial_port, baudrate)
        self.serial_interface.connect()

        # Publisher to send data to ROS
        self.publisher = self.create_publisher(String, 'serial_in', 10)

        # Subscriber to handle incoming waypoints from the waypoint publisher
        self.waypoint_subscription = self.create_subscription(
            String,
            'waypoints_topic',
            self.handle_incoming_waypoints,
            10
        )

        # Timer to receive data periodically
        self.timer = self.create_timer(0.1, self.receive_serial_data)

        self.waypoints = []  # List to store received waypoints
        self.current_waypoint = 0  # Pointer to the current waypoint
        self.waiting_for_ack = False  # Flag to check if we're waiting for an acknowledgment

    def handle_incoming_waypoints(self, msg):
        """Handles incoming waypoint messages and stores them for sending."""
        self.get_logger().info(f"Received waypoints: {msg.data}")

        if msg.data.startswith("WAYPOINTS"):
            waypoints_data = msg.data[len("WAYPOINTS"):].strip()
            waypoints = waypoints_data.split(";")  # Expecting each waypoint to be separated by a semicolon
            self.waypoints.clear()  # Clear previous waypoints

            for waypoint in waypoints:
                if waypoint == 'GO':
                    self.waypoints.append(f"GO")
                    break

                # Format each waypoint as "MOVE,<x>,<y>,<z>"
                x, y, z = waypoint.split(",")
                self.waypoints.append(f"MOVE,{x},{y},{z}")
            self.get_logger().info(f"Waypoints parsed and stored: {self.waypoints}")

            # Send the first waypoint to trigger sending
            self.send_next_waypoint()

    def send_next_waypoint(self):
        """Sends the next waypoint to the microcontroller."""
        if self.current_waypoint < len(self.waypoints):
            # Send the next waypoint if not already waiting for acknowledgment
            if not self.waiting_for_ack:
                msg = self.waypoints[self.current_waypoint]
                self.get_logger().info(f"Sending waypoint: {msg}")
                self.serial_interface.send_data(msg)
                self.waiting_for_ack = True  # Set flag to indicate we're waiting for acknowledgment
                self.current_waypoint += 1
            else:
                self.get_logger().info("Waiting for acknowledgment before sending the next waypoint.")
        else:
            self.get_logger().info("All waypoints sent. Sending GO signal to start robot.")
            self.send_go_signal()

    def send_go_signal(self):
        """Sends the GO signal to start the robot movement."""
        self.serial_interface.send_data("GO")
        self.get_logger().info("GO signal sent. Robot will start moving.")

    def receive_serial_data(self):
        """Checks for incoming serial data and publishes it to a ROS topic."""
        data = self.serial_interface.receive_data()
        if data:
            self.get_logger().info(f"Received from Arduino: {data}")
            self.publish_received_data(data)

            # Handle the acknowledgment to proceed with the next waypoint
            if "ACK" in data:  # Example acknowledgment message from robot (e.g., "ACK")
                self.get_logger().info("Acknowledgment received.")
                self.waiting_for_ack = False  # Reset the flag to allow sending the next waypoint
                self.send_next_waypoint()

    def publish_received_data(self, data):
        """Publishes received data to a ROS topic."""
        msg = String()
        msg.data = data
        self.publisher.publish(msg)

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
