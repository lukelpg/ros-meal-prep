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

        # Subscribe to stroke commands (can include multiple strokes separated by semicolons)
        self.create_subscription(String, 'paint_command', self.handle_command, 10)

    def handle_command(self, msg):
        """Processes a paint command to generate and publish coordinates.
        Supports multiple stroke commands separated by semicolons.
        """
        self.get_logger().info(f"Received command: {msg.data}")

        # Split incoming message into individual stroke commands
        stroke_commands = [cmd.strip() for cmd in msg.data.split(';') if cmd.strip()]
        all_stroke_points = []
        
        for stroke_command in stroke_commands:
            stroke_type, params, curve_depth = self.parse_command(stroke_command)
            self.get_logger().info(
                f"Parsed stroke type: {stroke_type}, Params: {params}, Depth: {curve_depth}"
            )
            stroke_points = self.stroke_generator.generate_stroke(
                stroke_type, params, curve_depth
            )
            self.get_logger().info(
                f"Generated {len(stroke_points)} stroke points for command '{stroke_command}': {stroke_points}"
            )

            if not stroke_points:
                self.get_logger().error("No valid stroke points generated for command: " + stroke_command)
                continue  # Process next stroke command if available

            all_stroke_points.extend(stroke_points)

        if not all_stroke_points:
            self.get_logger().error("No valid stroke points generated from any commands. Check stroke parameters.")
            return

        processed_curve = self.curve_processor.process_curve(all_stroke_points)
        coordinate_sequence = self.coordinate_compiler.compile_coordinates(processed_curve)
        self.instruction_publisher.publish_instructions(coordinate_sequence)

    def parse_command(self, command):
        """Parses a stroke command and extracts stroke type, parameters, and curve depth."""
        self.get_logger().info(f"Parsing command: {command}")
        parts = command.split(',')
        stroke_type = parts[0].strip()
        param_values = [float(x.strip()) for x in parts[1:]]

        # For arc strokes: expect at least 5 parameters (center_x, center_y, radius, start_angle, end_angle)
        # Optionally, the last parameter is the curve depth.
        if stroke_type.lower() == "arc":
            if len(param_values) >= 6:
                curve_depth = param_values.pop()  # Use last value as depth if provided
            else:
                curve_depth = 0  # Default depth if not specified

            center = (param_values[0], param_values[1])
            radius = param_values[2]
            start_angle = param_values[3]
            end_angle = param_values[4]
            params = [center, radius, start_angle, end_angle]
        else:
            # For line strokes, assume an even number of parameters represent points.
            # If odd, the last parameter is the curve depth.
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
