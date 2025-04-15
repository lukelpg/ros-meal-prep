# painting_pkg/painting_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from painting_pkg.stroke_generator import StrokeGenerator
from painting_pkg.curve_processor import CurveProcessor
from painting_pkg.coordinate_compiler import CoordinateCompiler
from painting_pkg.instruction_publisher import InstructionPublisher

class PaintingNode(Node):
    def __init__(self):
        super().__init__('painting_node')

        # Define workspace bounds.
        workspace_bounds = {
            "x": (0, -1000),
            "y": (0, -1000),
            "z": (0, 6000)
        }

        # Initialize components with workspace bounds.
        self.stroke_generator = StrokeGenerator(workspace_bounds)
        self.curve_processor = CurveProcessor()
        self.coordinate_compiler = CoordinateCompiler()
        self.instruction_publisher = InstructionPublisher()

        # Subscribe to stroke commands (multiple commands separated by semicolons).
        self.create_subscription(String, 'paint_command', self.handle_command, 10)
        self.strokes = []
        self.current_stroke_index = 0

    def handle_command(self, msg):
        """Processes a paint command to generate and publish coordinates."""
        self.get_logger().info(f"Received command: {msg.data}")
        # Split incoming message by semicolon.
        stroke_commands = [cmd.strip() for cmd in msg.data.split(';') if cmd.strip()]
        all_stroke_points = []
        
        for stroke_command in stroke_commands:
            stroke_type, params, curve_depth = self.parse_command(stroke_command)
            if stroke_type is None:
                continue
            self.get_logger().info(
                f"Parsed stroke type: {stroke_type}, Params: {params}, Depth: {curve_depth}"
            )
            stroke_points = self.stroke_generator.generate_stroke(
                stroke_type, params, curve_depth
            )
            self.get_logger().info(
                f"Generated {len(stroke_points)} stroke points for command '{stroke_command}'"
            )
            if not stroke_points:
                self.get_logger().error("No valid stroke points for command: " + stroke_command)
                continue
            all_stroke_points.extend(stroke_points)

        if not all_stroke_points:
            self.get_logger().error("No valid stroke points generated. Check parameters.")
            return

        processed_curve = self.curve_processor.process_curve(all_stroke_points)
        coordinate_sequence = self.coordinate_compiler.compile_coordinates(processed_curve)
        self.instruction_publisher.publish_instructions(coordinate_sequence)

    def parse_command(self, command):
        """Parses a stroke command and extracts stroke type, parameters, and curve depth."""
        self.get_logger().info(f"Parsing command: {command}")
        parts = command.split(',')
        stroke_type = parts[0].strip().lower()
        
        if stroke_type == "arc":
            if len(parts) >= 7:
                param_values = [float(x.strip()) for x in parts[1:-1]]
                curve_depth = float(parts[-1].strip())
            else:
                param_values = [float(x.strip()) for x in parts[1:]]
                curve_depth = 0
            center = (param_values[0], param_values[1])
            radius = param_values[2]
            start_angle = param_values[3]
            end_angle = param_values[4]
            params = [center, radius, start_angle, end_angle]
        elif stroke_type == "dip":
            if len(parts) < 8:
                self.get_logger().error("Dip command requires 7 parameters.")
                return None, None, None
            pickup = (float(parts[1].strip()), float(parts[2].strip()), float(parts[3].strip()))
            safe = (float(parts[4].strip()), float(parts[5].strip()), float(parts[6].strip()))
            color_hex = parts[7].strip()
            params = [pickup[0], pickup[1], pickup[2], safe[0], safe[1], safe[2], color_hex]
            curve_depth = 0
        else:
            param_values = [float(x.strip()) for x in parts[1:]]
            if len(param_values) % 2 == 1:
                curve_depth = param_values.pop()
            else:
                curve_depth = 0
            params = [tuple(param_values[i:i+2]) for i in range(0, len(param_values), 2)]
        
        self.get_logger().info(
            f"Extracted stroke type: {stroke_type}, Params: {params}, Depth: {curve_depth}"
        )
        return stroke_type, params, curve_depth

def main(args=None):
    rclpy.init(args=args)
    node = PaintingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
