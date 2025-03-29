# image_processing_pkg/stroke_mapping.py

import cv2
import numpy as np
import math

def generate_strokes(approx, shape_name, brush_width):
    """
    Generate strokes for the given shape:
      - For a "Circle", create concentric arcs.
      - For other shapes, fill the polygon with line strokes.
    Returns a list of stroke definition strings.
    """
    if shape_name == "Circle":
        return generate_circle_arcs(approx, brush_width)
    else:
        return generate_polygon_lines(approx, brush_width)

def generate_circle_arcs(approx, brush_width):
    """Generate concentric arc strokes for a circle."""
    strokes = []
    (cx, cy), radius = cv2.minEnclosingCircle(approx)
    cx, cy = int(cx), int(cy)
    max_r = int(radius)
    for r in range(0, max_r+1, brush_width):
        stroke = f"arc, {cx}, {cy}, {r}, 0, 360, {brush_width}"
        strokes.append(stroke)
    return strokes

def generate_polygon_lines(approx, brush_width):
    """
    For a polygon (non-circle), fill it with line strokes that run parallel to its longest edge.
    """
    pts = approx.reshape(-1, 2)
    # Determine longest edge and its angle
    longest_edge_length = 0
    longest_angle = 0
    n = len(pts)
    for i in range(n):
        p1 = pts[i]
        p2 = pts[(i+1) % n]
        edge_length = np.linalg.norm(p2 - p1)
        if edge_length > longest_edge_length:
            longest_edge_length = edge_length
            longest_angle = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
    # Rotate polygon so that the longest edge is horizontal
    cos_theta = math.cos(longest_angle)
    sin_theta = math.sin(longest_angle)
    R = np.array([[cos_theta, sin_theta],
                  [-sin_theta, cos_theta]])
    pts_rotated = np.dot(pts, R.T)
    min_y = np.min(pts_rotated[:,1])
    max_y = np.max(pts_rotated[:,1])
    strokes = []
    start_y = int(math.floor(min_y))
    end_y = int(math.ceil(max_y))
    # Generate horizontal scanlines in the rotated space
    for scan_y in range(start_y, end_y+1, brush_width):
        intersections = []
        for i in range(n):
            p1 = pts_rotated[i]
            p2 = pts_rotated[(i+1) % n]
            if (p1[1] <= scan_y and p2[1] > scan_y) or (p2[1] <= scan_y and p1[1] > scan_y):
                if abs(p2[1] - p1[1]) < 1e-6:
                    continue
                t = (scan_y - p1[1]) / (p2[1] - p1[1])
                intersect_x = p1[0] + t * (p2[0] - p1[0])
                intersections.append(intersect_x)
        intersections.sort()
        for j in range(0, len(intersections)-1, 2):
            x_left = intersections[j]
            x_right = intersections[j+1]
            pt1_rot = np.array([x_left, scan_y])
            pt2_rot = np.array([x_right, scan_y])
            R_inv = np.array([[cos_theta, -sin_theta],
                              [sin_theta, cos_theta]])
            pt1_orig = np.dot(R_inv, pt1_rot)
            pt2_orig = np.dot(R_inv, pt2_rot)
            pt1 = (int(round(pt1_orig[0])), int(round(pt1_orig[1])))
            pt2 = (int(round(pt2_orig[0])), int(round(pt2_orig[1])))
            stroke = f"line, {pt1[0]}, {pt1[1]}, {pt2[0]}, {pt2[1]}, {brush_width}"
            strokes.append(stroke)
    return strokes
