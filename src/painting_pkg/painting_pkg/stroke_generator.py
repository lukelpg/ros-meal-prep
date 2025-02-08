import numpy as np

class StrokeGenerator:
    def __init__(self, workspace_bounds):
        self.x_min, self.x_max = workspace_bounds["x"]
        self.y_min, self.y_max = workspace_bounds["y"]
        self.z_min, self.z_max = workspace_bounds["z"]

    def is_within_workspace(self, x, y, z=0):
        """Checks if a coordinate is within the workspace."""
        return (self.x_min >= x >= self.x_max and  # X decreases from 0 to -1000
                self.y_min >= y >= self.y_max and  # Y decreases from 0 to -1000
                self.z_min <= z <= self.z_max)

    def generate_stroke(self, stroke_type, params, curve_depth=0):
        """Generates stroke points before curve processing."""
        print(f"Generating {stroke_type} stroke with params: {params}, Depth: {curve_depth}")

        if stroke_type == "line":
            points = self._generate_line(*params, curve_depth)
        elif stroke_type == "arc":
            points = self._generate_arc(*params)
        else:
            print(f"Unknown stroke type: {stroke_type}")
            return []

        print(f"Generated {len(points)} stroke points before filtering.")

        valid_points = [p for p in points if self.is_within_workspace(p[0], p[1])]
        print(f"Remaining {len(valid_points)} valid points after workspace filtering.")

        return valid_points

    def _generate_line(self, start, end, curve_depth=100, num_points=10):
        """
        Generates a line with a downward curve:
        
        - Starts at `Z = curve_depth` (default 100).
        - Dips to `Z = 0` at the midpoint.
        - Rises back to `Z = curve_depth` at the end.
        """
        print(f"Generating downward curved stroke from {start} to {end}, Depth: {curve_depth}")

        x_vals = np.linspace(start[0], end[0], num_points)
        y_vals = np.linspace(start[1], end[1], num_points)

        # Create a downward curve by flipping the previous equation
        z_vals = [curve_depth * (1 - (2 * (i / (num_points - 1)) - 1) ** 2) for i in range(num_points)]

        # Invert so that the stroke **starts at Z = 100, dips to Z = 0, and rises back**
        z_vals = [curve_depth - z for z in z_vals]

        return [(x, y, z) for x, y, z in zip(x_vals, y_vals, z_vals)]

    def _generate_arc(self, center, radius, start_angle, end_angle, num_points=20):
        """Generates points along an arc."""
        print(f"Generating arc at {center} with radius {radius} from {start_angle}° to {end_angle}°")

        angles = np.linspace(np.radians(start_angle), np.radians(end_angle), num_points)
        return [(center[0] + radius * np.cos(a), center[1] + radius * np.sin(a)) for a in angles]
