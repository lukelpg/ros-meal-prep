import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')

        # Publisher to send waypoints
        self.publisher = self.create_publisher(String, 'waypoints_topic', 10)

        # Define waypoints (can be made dynamic as well)
        self.waypoints = [
            "-200, -200, 0",
            "-800, -200, 0",
            "-800, -800, 0",
            "-200, -800, 0",
            "-200, -200, 0"
        ]

        # Publish waypoints once
        self.publish_waypoints()

    def publish_waypoints(self):
        """Publishes the waypoints as a single message."""
        waypoint_message = "WAYPOINTS " + ";".join(self.waypoints)
        msg = String()
        msg.data = waypoint_message
        self.publisher.publish(msg)
        self.get_logger().info(f"Published waypoints: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = WaypointPublisher()

    try:
        rclpy.spin(node)  # Keep the node alive, no timer needed
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
