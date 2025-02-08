import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class StrokeTestNode(Node):
    def __init__(self):
        super().__init__('stroke_test_node')
        self.publisher = self.create_publisher(String, 'paint_command', 10)

    def send_stroke(self, stroke_command):
        msg = String()
        msg.data = stroke_command
        self.publisher.publish(msg)
        self.get_logger().info(f"Sent stroke command: {stroke_command}")

def main():
    rclpy.init()
    node = StrokeTestNode()
    time.sleep(1)  # Wait for ROS 2 to initialize

    # Send test strokes
    test_strokes = [
        # "line, 0, 0, -1000, -1000, 500",
        # "line, -200, -200, -800, -800",
        "arc, -500, -500, 500, 0, 180, 1000"
    ]

    for stroke in test_strokes:
        node.send_stroke(stroke)
        time.sleep(2)  # Wait before sending next stroke

    rclpy.shutdown()

if __name__ == "__main__":
    main()
