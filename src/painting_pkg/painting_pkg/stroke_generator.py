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
        """Generates stroke points before curve processing, ensuring depth is used."""
        print(f"Generating {stroke_type} stroke with params: {params}, Depth: {curve_depth}")

        if stroke_type.lower() == "line":
            points = self._generate_line(*params, curve_depth)
        elif stroke_type.lower() == "arc":
            points = self._generate_arc(*params, curve_depth)
        elif stroke_type.lower() == "dip":
            points = self._generate_dip(params)
        else:
            print(f"Unknown stroke type: {stroke_type}")
            return []

        print(f"Generated {len(points)} stroke points before filtering.")
        
        valid_points = [p for p in points if self.is_within_workspace(p[0], p[1], p[2])]
        print(f"Remaining {len(valid_points)} valid points after workspace filtering.")

        return valid_points

    def _generate_line(self, start, end, curve_depth=100, num_points=10):
        """
        Generates a line with a downward curve:
         - Starts at Z = curve_depth,
         - Dips to Z = 0 at the midpoint,
         - Rises back to Z = curve_depth.
        """
        print(f"Generating downward curved stroke from {start} to {end}, Depth: {curve_depth}")
        x_vals = np.linspace(start[0], end[0], num_points)
        y_vals = np.linspace(start[1], end[1], num_points)
        z_vals = [curve_depth * (1 - (2 * (i / (num_points - 1)) - 1)**2) for i in range(num_points)]
        z_vals = [curve_depth - z for z in z_vals]
        return [(x, y, z) for x, y, z in zip(x_vals, y_vals, z_vals)]

    def _generate_arc(self, center, radius, start_angle, end_angle, curve_depth, num_points=20):
        """
        Generates an arc with a downward curve:
         - Starts at Z = curve_depth,
         - Dips to Z = 0,
         - Rises back to Z = curve_depth.
        """
        print(f"Generating arc at {center} with radius {radius} from {start_angle}° to {end_angle}° with depth {curve_depth}")
        angles = np.linspace(np.radians(start_angle), np.radians(end_angle), num_points)
        x_vals = center[0] + radius * np.cos(angles)
        y_vals = center[1] + radius * np.sin(angles)
        z_vals = [curve_depth * (1 - (2 * (i / (num_points - 1)) - 1)**2) for i in range(num_points)]
        z_vals = [curve_depth - z for z in z_vals]
        return [(x, y, z) for x, y, z in zip(x_vals, y_vals, z_vals)]

    def _generate_dip(self, params, num_points=5):
        """
        Generates a dip stroke trajectory.
        Expected params (list): 
            [pickup_x, pickup_y, pickup_z, safe_x, safe_y, safe_z, color_hex]
        The function will generate a simple trajectory:
         - From safe coordinate to pickup coordinate and back to safe coordinate.
        """
        print(f"Generating dip stroke with params: {params}")
        try:
            pickup_x = float(params[0])
            pickup_y = float(params[1])
            pickup_z = float(params[2])
            safe_x   = float(params[3])
            safe_y   = float(params[4])
            safe_z   = float(params[5])
            # color_hex is params[6] but is not used for trajectory computation.
        except (ValueError, IndexError) as e:
            print("Invalid parameters for dip stroke:", e)
            return []

        pickup = (pickup_x, pickup_y, pickup_z)
        safe = (safe_x, safe_y, safe_z)

        # Generate trajectory: go from safe to pickup, then back to safe.
        line_to_pickup = [(
            safe[0] + (pickup[0] - safe[0]) * t,
            safe[1] + (pickup[1] - safe[1]) * t,
            safe[2] + (pickup[2] - safe[2]) * t
        ) for t in np.linspace(0, 1, num_points)]
        line_to_safe = [(
            pickup[0] + (safe[0] - pickup[0]) * t,
            pickup[1] + (safe[1] - pickup[1]) * t,
            pickup[2] + (safe[2] - pickup[2]) * t
        ) for t in np.linspace(0, 1, num_points)]
        # Avoid duplicate pickup point.
        trajectory = line_to_pickup + line_to_safe[1:]
        return trajectory
