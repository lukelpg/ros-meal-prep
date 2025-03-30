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

        # Current batch state
        self.waypoints = []          # List to store the active batch of waypoints
        self.current_waypoint = 0    # Pointer into the active batch
        self.waiting_for_ack = False # Flag to indicate waiting for an ACK

        # Queue for incoming waypoint batches
        self.waypoint_queue = []     

    def handle_incoming_waypoints(self, msg):
        """Handles incoming waypoint messages and queues them for sending."""
        self.get_logger().info(f"Received waypoints: {msg.data}")

        if msg.data.startswith("WAYPOINTS"):
            waypoints_data = msg.data[len("WAYPOINTS"):].strip()
            # Split into individual waypoint strings (assumes semicolon-separated)
            waypoints_list = waypoints_data.split(";")
            new_batch = []
            for waypoint in waypoints_list:
                waypoint = waypoint.strip()
                if not waypoint:
                    continue
                if waypoint.upper() == 'GO':
                    new_batch.append("GO")
                    break
                # Expect each waypoint in the form "x,y,z"
                try:
                    x, y, z = waypoint.split(",")
                    new_batch.append(f"MOVE,{x.strip()},{y.strip()},{z.strip()}")
                except ValueError:
                    self.get_logger().error(f"Bad waypoint format: {waypoint}")
            self.get_logger().info(f"New batch parsed: {new_batch}")

            # If no current batch is active (i.e. finished processing) then load this batch.
            if not self.waypoints and not self.waiting_for_ack:
                self.waypoints = new_batch
                self.current_waypoint = 0
                self.waiting_for_ack = False
                self.send_next_waypoint()
            else:
                # Otherwise, queue the new batch.
                self.waypoint_queue.append(new_batch)
                self.get_logger().info("Current batch in progress; queued new batch.")

    def send_next_waypoint(self):
        """Sends the next waypoint from the active batch to the microcontroller."""
        if self.current_waypoint < len(self.waypoints):
            if not self.waiting_for_ack:
                msg = self.waypoints[self.current_waypoint]
                self.get_logger().info(f"Sending waypoint: {msg}")
                self.serial_interface.send_data(msg)
                self.waiting_for_ack = True
                self.current_waypoint += 1
            else:
                self.get_logger().info("Waiting for acknowledgment before sending the next waypoint.")
        else:
            self.get_logger().info("All waypoints in current batch sent. Sending GO signal.")
            self.send_go_signal()
            # After sending GO, load the next batch from the queue if available.
            if self.waypoint_queue:
                next_batch = self.waypoint_queue.pop(0)
                self.get_logger().info(f"Loading next batch: {next_batch}")
                self.waypoints = next_batch
                self.current_waypoint = 0
                self.waiting_for_ack = False
                # Start sending the new batch.
                self.send_next_waypoint()
            else:
                # No new batch queued; reset state.
                self.waypoints = []
                self.current_waypoint = 0
                self.waiting_for_ack = False

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
            # If an acknowledgment is received, clear the waiting flag and send the next waypoint.
            if "ACK" in data:
                self.get_logger().info("Acknowledgment received.")
                self.waiting_for_ack = False
                self.send_next_waypoint()
            # If the DONE signal is received, clear the current batch and load the next queued batch if available.
            if "DONE" in data:
                self.get_logger().info("DONE received. Finishing current batch and starting next batch if available.")
                self.waiting_for_ack = False
                # Clear current batch (assuming DONE signals that the current batch is complete)
                self.waypoints = []
                self.current_waypoint = 0
                if self.waypoint_queue:
                    next_batch = self.waypoint_queue.pop(0)
                    self.get_logger().info(f"Loading next batch: {next_batch}")
                    self.waypoints = next_batch
                    self.send_next_waypoint()
                else:
                    self.get_logger().info("No new waypoint batch queued.")

    def publish_received_data(self, data):
        """Publishes received data to a ROS topic."""
        msg = String()
        msg.data = data
        self.publisher.publish(msg)

    def on_shutdown(self):
        """Handles shutdown and closes the serial connection."""
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

if __name__ == '__main__':
    main()
