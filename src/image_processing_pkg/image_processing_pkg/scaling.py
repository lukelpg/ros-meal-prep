# image_processing_pkg/scaling.py

def scale_point(pt, image_dims, workspace_bounds):
    """
    Maps a point (x, y) from image coordinates to workspace coordinates.
    """
    width, height = image_dims
    ws_x_min, ws_x_max = min(workspace_bounds["x"]), max(workspace_bounds["x"])
    ws_y_min, ws_y_max = min(workspace_bounds["y"]), max(workspace_bounds["y"])
    x, y = pt
    scaled_x = ws_x_min + (x / width) * (ws_x_max - ws_x_min)
    scaled_y = ws_y_min + (y / height) * (ws_y_max - ws_y_min)
    return (scaled_x, scaled_y)

def scale_stroke(stroke, image_dims, workspace_bounds):
    """
    Given a stroke definition string ("line" or "arc"), scale its coordinates to the workspace.
    Returns a new stroke definition string with scaled coordinates.
    """
    parts = [p.strip() for p in stroke.split(',')]
    cmd = parts[0].lower()
    if cmd == "line":
        x1 = float(parts[1])
        y1 = float(parts[2])
        x2 = float(parts[3])
        y2 = float(parts[4])
        brush = parts[5]
        pt1 = scale_point((x1, y1), image_dims, workspace_bounds)
        pt2 = scale_point((x2, y2), image_dims, workspace_bounds)
        return f"line, {int(round(pt1[0]))}, {int(round(pt1[1]))}, {int(round(pt2[0]))}, {int(round(pt2[1]))}, {brush}"
    elif cmd == "arc":
        cx = float(parts[1])
        cy = float(parts[2])
        r = float(parts[3])
        start_angle = parts[4]
        end_angle = parts[5]
        brush = parts[6]
        pt_center = scale_point((cx, cy), image_dims, workspace_bounds)
        ws_x_min, ws_x_max = min(workspace_bounds["x"]), max(workspace_bounds["x"])
        scale_x = (ws_x_max - ws_x_min) / image_dims[0]
        scaled_r = r * scale_x
        return f"arc, {int(round(pt_center[0]))}, {int(round(pt_center[1]))}, {int(round(scaled_r))}, {start_angle}, {end_angle}, {brush}"
    else:
        return stroke
