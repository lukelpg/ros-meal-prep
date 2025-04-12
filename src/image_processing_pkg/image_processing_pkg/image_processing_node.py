# image_processing_pkg/image_processing_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from image_processing_pkg.image_processor import detect_shapes
from image_processing_pkg.config import workspace_bounds
from image_processing_pkg.scaling import scale_stroke
from image_processing_pkg.color_utils import get_closest_palette_color, generate_dip_command, palette

class ImageProcessingNode(Node):
    def __init__(self):
        super().__init__('image_processing_node')
        self.publisher_ = self.create_publisher(String, 'paint_command', 10)
        self.get_logger().info("Starting image processing...")

        # Process the image to generate strokes and get image dimensions.
        # Each stroke is now a tuple: (stroke_definition, shape_color)
        self.strokes, self.image_dims = detect_shapes("blue_x.png", epsilon_factor=0.01, min_area=100)
        
        self.current_stroke_index = 0
        self.current_dip_commands = []  # Queue for dip commands
        self.current_color = None  # Current tool color (as a palette key)
        
        # Publish one command every 0.05 seconds (adjust as needed)
        self.timer = self.create_timer(0.05, self.publish_next_command)

    def publish_next_command(self):
        # If dip commands are queued, publish them first.
        if self.current_dip_commands:
            cmd = self.current_dip_commands.pop(0)
            self.publisher_.publish(String(data=cmd))
            self.get_logger().info(f"Published dip command: {cmd}")
            return

        if self.current_stroke_index < len(self.strokes):
            stroke_str, shape_color = self.strokes[self.current_stroke_index]
            # Convert the detected shape color (RGB tuple) to the closest palette color key.
            required_color_key = get_closest_palette_color(shape_color)
            
            # If the current tool color is not what’s needed, queue dip commands first.
            if self.current_color != required_color_key:
                # Get palette coordinates for the required color.
                palette_coord = palette[required_color_key]["coord"]
                # Here, we assume a placeholder current position; in a real system,
                # track your robot's actual current position.
                current_position = (0, 0, workspace_bounds["z"][0])
                self.current_dip_commands.extend(generate_dip_command(current_position, palette_coord))
                self.current_color = required_color_key
                self.get_logger().info(f"Color change: switching tool color to {required_color_key}. Dip commands queued.")
                return

            # Scale stroke coordinates from image space to workspace coordinates.
            scaled_stroke = scale_stroke(stroke_str, self.image_dims, workspace_bounds)
            self.publisher_.publish(String(data=scaled_stroke))
            self.get_logger().info(f"Published stroke command: {scaled_stroke}")
            self.current_stroke_index += 1
        else:
            self.get_logger().info("All commands published. Shutting down node.")
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
