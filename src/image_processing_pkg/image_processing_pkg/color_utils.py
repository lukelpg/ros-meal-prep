import math
from image_processing_pkg.config import palette, SAFE_Z, DIP_Z

def hex_to_rgb(hex_str):
    """Convert a hex color string (e.g., '#FF0000') to an (R, G, B) tuple."""
    hex_str = hex_str.lstrip('#')
    return tuple(int(hex_str[i:i+2], 16) for i in (0, 2, 4))

def color_distance(c1, c2):
    """Euclidean distance between two RGB colors."""
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(c1, c2)))

def get_closest_palette_color(shape_color):
    """
    Given a detected shape color in RGB, return the key (name) of the closest palette color.
    """
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
    Given the current 3D position (x, y, z) and the target palette coordinate (x, y),
    generate a list of commands to perform the dip motion.
    Commands are formatted as strings, for example:
      - "move_to, x, y, z"
      - "dip" (for the actual dipping action)
    """
    commands = []
    
    # 1. Raise to the safe Z height if not already at or above it
    if current_pos[2] < SAFE_Z:
        commands.append(f"move_to, {current_pos[0]}, {current_pos[1]}, {SAFE_Z}")
    
    # 2. Move horizontally to above the palette coordinate at safe Z
    commands.append(f"move_to, {palette_coord[0]}, {palette_coord[1]}, {SAFE_Z}")
    
    # 3. Lower down to DIP_Z to dip into the color
    commands.append(f"move_to, {palette_coord[0]}, {palette_coord[1]}, {DIP_Z}")
    
    # 4. Issue a dip command (could include a dwell or activation)
    commands.append("dip")
    
    # 5. Raise back to the safe height
    commands.append(f"move_to, {palette_coord[0]}, {palette_coord[1]}, {SAFE_Z}")
    
    return commands
