import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from image_processing_pkg.image_processor import detect_shapes
from image_processing_pkg.config import inputImage, workspace_bounds
from image_processing_pkg.scaling import scale_stroke

class ImageProcessingNode(Node):
    def __init__(self):
        super().__init__('image_processing_node')
        self.publisher_ = self.create_publisher(String, 'paint_command', 10)
        self.get_logger().info("Starting image processing...")

        # Process the image to generate raw stroke definitions and get image dimensions
        raw_strokes, image_dims = detect_shapes(inputImage, epsilon_factor=0.01, min_area=100)
        
        # Scale each stroke before publishing using the scale_stroke function
        self.strokes = [scale_stroke(stroke, image_dims, workspace_bounds) for stroke in raw_strokes]
        self.current_stroke = 0

        # Publish one scaled stroke command every second
        self.timer = self.create_timer(0.01, self.publish_next_stroke)

    def publish_next_stroke(self):
        if self.current_stroke < len(self.strokes):
            stroke_msg = String()
            stroke_msg.data = self.strokes[self.current_stroke]
            self.publisher_.publish(stroke_msg)
            self.get_logger().info(f"Published scaled stroke: {stroke_msg.data}")
            self.current_stroke += 1
        else:
            self.get_logger().info("All strokes published. Shutting down node.")
            self.timer.cancel()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
