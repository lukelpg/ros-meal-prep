# image_processing_pkg/image_processing_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from image_processing_pkg.image_processor import detect_shapes
from image_processing_pkg.config import workspace_bounds, inputImage
from image_processing_pkg.scaling import scale_stroke

class ImageProcessingNode(Node):
    def __init__(self):
        super().__init__('image_processing_node')
        self.publisher_ = self.create_publisher(String, 'paint_command', 10)
        self.get_logger().info("Starting image processing...")

        # Grab the strokes (with "dip,..." already at each shape’s start)
        # Each entry is (stroke_string, shape_color)
        self.strokes, self.image_dims = detect_shapes(
            inputImage, epsilon_factor=0.01, min_area=100
        )
        self.index = 0

        # Publish one command every 0.05s
        self.timer = self.create_timer(0.05, self.publish_next)

    def publish_next(self):
        if self.index >= len(self.strokes):
            self.get_logger().info("All strokes published. Shutting down node.")
            self.timer.cancel()
            rclpy.shutdown()
            return

        stroke_str, _ = self.strokes[self.index]

        # Let your inserted dip commands pass unchanged
        if stroke_str.strip().lower().startswith("dip"):
            cmd = stroke_str
        else:
            # Scale only line/arc strokes
            cmd = scale_stroke(stroke_str, self.image_dims, workspace_bounds)

        self.publisher_.publish(String(data=cmd))
        self.get_logger().info(f"Published command: {cmd}")

        self.index += 1

def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
