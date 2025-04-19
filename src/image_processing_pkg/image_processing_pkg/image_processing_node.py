# image_processing_pkg/image_processing_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from image_processing_pkg.image_processor import detect_shapes
from image_processing_pkg.config import (
    inputImage, workspace_bounds,
    DIP_INTERVAL, DIP_Z, SAFE_Z, palette
)
from image_processing_pkg.scaling import scale_stroke
from image_processing_pkg.color_utils import get_closest_palette_color

class ImageProcessingNode(Node):
    def __init__(self):
        super().__init__('image_processing_node')
        self.pub = self.create_publisher(String, 'paint_command', 10)
        self.get_logger().info("Starting image processing...")

        # Flat list of (stroke_str, shape_color)
        self.strokes, self.image_dims = detect_shapes(
            inputImage, epsilon_factor=0.01, min_area=100
        )

        self.index = 0
        self.waiting_for_dip = False

        # Publish every 0.05s
        self.timer = self.create_timer(0.05, self.publish_next)

    def publish_next(self):
        # Done?
        if self.index >= len(self.strokes):
            self.get_logger().info("All strokes done. Shutting down.")
            self.timer.cancel()
            rclpy.shutdown()
            return

        stroke_def, shape_color = self.strokes[self.index]

        # Should we dip before this stroke?
        if self.index % DIP_INTERVAL == 0 and not self.waiting_for_dip:
            # Build dip command for this shape color
            key = get_closest_palette_color(shape_color)
            x, y = palette[key]['coord']
            hexcol = palette[key]['hex']
            dip_cmd = f"dip, {x}, {y}, {DIP_Z}, {x}, {y}, {SAFE_Z}, {hexcol}"

            self.pub.publish(String(data=dip_cmd))
            self.get_logger().info(f"Published dip: {dip_cmd}")

            # Mark that we've done the dip; next tick will send the stroke itself
            self.waiting_for_dip = True
            return

        # Otherwise, send the actual stroke
        # (if it somehow is a dip text, just pass through; else scale)
        if stroke_def.strip().lower().startswith("dip"):
            cmd = stroke_def
        else:
            cmd = scale_stroke(stroke_def, self.image_dims, workspace_bounds)

        self.pub.publish(String(data=cmd))
        self.get_logger().info(f"Published stroke: {cmd}")

        # Advance to next stroke and reset dip flag
        self.index += 1
        self.waiting_for_dip = False

def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
