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
        self.pub = self.create_publisher(String, 'paint_command', 10)
        self.get_logger().info("Starting image processing...")

        # 1) Run shape detection, which already inserts a "dip" command at the start of each shape
        strokes, self.image_dims = detect_shapes(inputImage,
                                                 epsilon_factor=0.01,
                                                 min_area=100)

        # 2) Group into per‐shape lists: every time we see a "dip" we start a new group
        self.shape_commands = []
        current = []
        for stroke_str, _color in strokes:
            if stroke_str.strip().lower().startswith("dip"):
                if current:
                    self.shape_commands.append(current)
                current = [stroke_str]
            else:
                current.append(stroke_str)
        if current:
            self.shape_commands.append(current)

        self.shape_index = 0

        # 3) Publish one shape‐batch every 0.05s
        self.timer = self.create_timer(0.05, self.publish_next_shape)

    def publish_next_shape(self):
        if self.shape_index >= len(self.shape_commands):
            self.get_logger().info("All shapes published. Shutting down.")
            self.timer.cancel()
            rclpy.shutdown()
            return

        batch = self.shape_commands[self.shape_index]
        out_cmds = []
        for stroke_str in batch:
            if stroke_str.strip().lower().startswith("dip"):
                # pass the dip command unchanged (already in workspace coords)
                out_cmds.append(stroke_str)
            else:
                # scale only line/arc strokes
                out_cmds.append(scale_stroke(stroke_str,
                                             self.image_dims,
                                             workspace_bounds))

        msg = String()
        msg.data = ";".join(out_cmds)
        self.pub.publish(msg)
        self.get_logger().info(f"Published paint_command: {msg.data}")

        self.shape_index += 1

def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
