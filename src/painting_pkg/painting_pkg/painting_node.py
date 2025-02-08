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

        # Define workspace bounds
        workspace_bounds = {
            "x": (0, -1000),
            "y": (0, -1000),
            "z": (0, 6000)
        }

        # Initialize components with workspace bounds
        self.stroke_generator = StrokeGenerator(workspace_bounds)
        self.curve_processor = CurveProcessor()
        self.coordinate_compiler = CoordinateCompiler()
        self.instruction_publisher = InstructionPublisher()

        # Subscribe to stroke commands
        self.create_subscription(String, 'paint_command', self.handle_command, 10)

    def handle_command(self, msg):
        """Processes a paint command to generate and publish coordinates."""
        self.get_logger().info(f"Received command: {msg.data}")

        # Fix: Ensure all three values are received
        stroke_type, params, curve_depth = self.parse_command(msg.data)
        self.get_logger().info(f"Parsed stroke type: {stroke_type}, Params: {params}, Depth: {curve_depth}")

        stroke_points = self.stroke_generator.generate_stroke(stroke_type, params, curve_depth)
        self.get_logger().info(f"Generated {len(stroke_points)} stroke points: {stroke_points}")

        if not stroke_points:
            self.get_logger().error("No valid stroke points generated. Check stroke parameters.")
            return  # Stop processing if stroke points are empty

        processed_curve = self.curve_processor.process_curve(stroke_points)
        coordinate_sequence = self.coordinate_compiler.compile_coordinates(processed_curve)
        self.instruction_publisher.publish_instructions(coordinate_sequence)


    def parse_command(self, command):
        """Parses incoming stroke commands and extracts parameters."""
        self.get_logger().info(f"Parsing command: {command}")
        parts = command.split(',')

        stroke_type = parts[0]
        param_values = list(map(float, parts[1:]))

        # Arc strokes need 5 params: center_x, center_y, radius, start_angle, end_angle
        if stroke_type == "arc":
            if len(param_values) == 6:
                curve_depth = param_values.pop()  # Extract last value as depth
            else:
                curve_depth = 0  # Default depth if not specified

            center = (param_values[0], param_values[1])
            radius = param_values[2]
            start_angle = param_values[3]
            end_angle = param_values[4]
            params = [center, radius, start_angle, end_angle]

        # Line strokes need at least 2 points
        else:
            if len(param_values) % 2 == 1:
                curve_depth = param_values.pop()
            else:
                curve_depth = 0  # Default depth

            params = [tuple(param_values[i:i+2]) for i in range(0, len(param_values), 2)]

        self.get_logger().info(f"Extracted stroke type: {stroke_type}, Params: {params}, Depth: {curve_depth}")
        return stroke_type, params, curve_depth

def main(args=None):
    rclpy.init(args=args)
    node = PaintingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
