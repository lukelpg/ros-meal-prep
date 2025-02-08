import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class InstructionPublisher(Node):
    def __init__(self):
        super().__init__('instruction_publisher')
        self.publisher = self.create_publisher(String, 'waypoints_topic', 10)

    def publish_instructions(self, instructions):
        """Publishes the compiled coordinate sequence as one message."""
        msg = String()
        msg.data = "WAYPOINTS " + ";".join(instructions)
        self.publisher.publish(msg)
        self.get_logger().info(f"Published waypoints: {msg.data}")
