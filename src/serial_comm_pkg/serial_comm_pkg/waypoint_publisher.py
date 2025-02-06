import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import math

class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')

        # Publisher to send waypoints
        self.publisher = self.create_publisher(String, 'waypoints_topic', 10)

        # Define square waypoints
        self.square_waypoints = [
            "-200, -200, 0",
            "-800, -200, 0",
            "-800, -800, 0",
            "-200, -800, 0",
            "-200, -200, 0",
            "GO"
        ]

        # Calculate the diameter of the circle (the diagonal of the square)
        # side_length = abs(self.square_waypoints[1][0] - self.square_waypoints[0][0])  # horizontal side length
        # circle_diameter = math.sqrt(2) * side_length  # Diagonal = side_length * sqrt(2)

        # Generate waypoints for the spiral
        # self.spiral_waypoints = self.generate_spiral_points(circle_diameter, 150, 6000)  # 360 points, 6000 units for z

        # Combine square and spiral waypoints
        # self.waypoints = self.spiral_waypoints + ["GO"]
        self.waypoints = self.square_waypoints

        # Publish waypoints once
        self.publish_waypoints()

    def generate_spiral_points(self, diameter, num_points, max_z):
        """Generate points for a spiral with a given diameter and max z value."""
        radius = diameter / 2
        center_x, center_y = -500, -500  # Center of the circle
        z_increment = max_z / num_points  # Divide the total z range by the number of points

        points = []
        for i in range(num_points):
            # Angle for the current point (evenly spaced)
            angle = 2 * math.pi * i / num_points
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            z = z_increment * i  # Increase z with each point
            points.append(f"{x}, {y}, {z}")

        return points

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
