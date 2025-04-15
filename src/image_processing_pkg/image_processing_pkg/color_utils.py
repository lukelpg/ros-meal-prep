# image_processing_pkg/movement_commands.py

import math
from image_processing_pkg.config import palette, SAFE_Z, DIP_Z

def hex_to_rgb(hex_str):
    """Convert a hex color string to (R, G, B)."""
    hex_str = hex_str.lstrip('#')
    return tuple(int(hex_str[i:i+2], 16) for i in (0, 2, 4))

def color_distance(c1, c2):
    """Euclidean distance between two RGB colors."""
    return math.sqrt(sum((a - b)**2 for a, b in zip(c1, c2)))

def get_closest_palette_color(shape_color):
    """Return the palette key for the closest color to shape_color."""
    best_match = None
    min_dist = float("inf")
    for key, props in palette.items():
        palette_rgb = hex_to_rgb(props["hex"])
        dist = color_distance(shape_color, palette_rgb)
        if dist < min_dist:
            min_dist = dist
            best_match = key
    return best_match

def generate_dip_command(current_pos, palette_coord):
    """
    Generates a list of motion commands for dipping.
    (Note: In our new design the dip stroke is inserted by the image processing package.)
    """
    commands = []
    if current_pos[2] < SAFE_Z:
        commands.append(f"move_to, {current_pos[0]}, {current_pos[1]}, {SAFE_Z}")
    commands.append(f"move_to, {palette_coord[0]}, {palette_coord[1]}, {SAFE_Z}")
    commands.append(f"move_to, {palette_coord[0]}, {palette_coord[1]}, {DIP_Z}")
    commands.append("dip")
    commands.append(f"move_to, {palette_coord[0]}, {palette_coord[1]}, {SAFE_Z}")
    return commands
