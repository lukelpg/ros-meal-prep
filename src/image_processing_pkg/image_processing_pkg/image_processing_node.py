# image_processing_pkg/image_processing_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from image_processing_pkg.image_processor import detect_shapes
from image_processing_pkg.config import (
    inputImage,
    workspace_bounds,
    DIP_Z,
    SAFE_Z,
    palette,
    DIP_INTERVAL  # ensure this is defined in config.py
)
from image_processing_pkg.scaling import scale_stroke
from image_processing_pkg.color_utils import get_closest_palette_color

class ImageProcessingNode(Node):
    def __init__(self):
        super().__init__('image_processing_node')
        self.pub = self.create_publisher(String, 'paint_command', 10)
        self.get_logger().info("Starting image processing...")

        # get per‑shape stroke groups
        self.shape_batches, self.image_dims = detect_shapes(
            inputImage, epsilon_factor=0.01, min_area=100
        )
        self.current = 0
        self.timer = self.create_timer(0.05, self.publish_next_shape)

    def publish_next_shape(self):
        if self.current >= len(self.shape_batches):
            self.get_logger().info("All shapes published, shutting down.")
            self.timer.cancel()
            rclpy.shutdown()
            return

        batch = self.shape_batches[self.current]
        # initial dip for this shape
        shape_color = batch[0][1]
        color_key = get_closest_palette_color(shape_color)
        px, py = palette[color_key]['coord']
        hexcol = palette[color_key]['hex']
        initial_dip = f"dip, {px}, {py}, {DIP_Z}, {px}, {py}, {SAFE_Z}, {hexcol}"

        msgs = [initial_dip]

        # insert strokes, with additional dips every DIP_INTERVAL strokes
        for idx, (stroke_def, _) in enumerate(batch, 1):
            # every DIP_INTERVAL strokes (not counting the initial dip), insert another dip
            if idx % DIP_INTERVAL == 0:
                msgs.append(initial_dip)
            # then the stroke itself
            if stroke_def.lower().startswith("dip"):
                msgs.append(stroke_def)
            else:
                msgs.append(scale_stroke(stroke_def, self.image_dims, workspace_bounds))

        data = ";".join(msgs)
        msg = String(data=data)
        self.pub.publish(msg)
        self.get_logger().info(f"Published shape {self.current} batch: {data}")

        self.current += 1

def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
