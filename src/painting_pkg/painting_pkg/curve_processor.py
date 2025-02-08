import numpy as np
from scipy.interpolate import CubicSpline

class CurveProcessor:
    def __init__(self):
        pass

    def process_curve(self, stroke_points):
        """Interpolates a curve from given stroke points."""
        print(f"Processing curve for {len(stroke_points)} points")

        # Unpack x, y, z separately
        x_vals, y_vals, z_vals = zip(*stroke_points)
        
        # Normalize time values
        t_vals = np.linspace(0, 1, len(stroke_points))

        # Use Cubic Spline for smooth interpolation
        spline_x = CubicSpline(t_vals, x_vals)
        spline_y = CubicSpline(t_vals, y_vals)
        spline_z = CubicSpline(t_vals, z_vals)

        # Generate new smooth points
        smooth_points = [
            (spline_x(t), spline_y(t), spline_z(t))
            for t in np.linspace(0, 1, len(stroke_points) * 2)
        ]

        return smooth_points
