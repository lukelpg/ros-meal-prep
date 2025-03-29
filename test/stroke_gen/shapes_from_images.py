import cv2
import numpy as np
import random
import math

# File names and workspace settings
inputImage = "linux.png"
edgesImage = "roughEdges.png"
closedEdgesImage = "closedEdges.png"
isolatedContours = "isolatedContours.png"
shapesOnImage = "shapesOnImage.png"
colouredShapes = "colouredShapes.png"
thinStrokesImage = "thinStrokes.png"

# Workspace bounds for x and y (z is ignored for stroke mapping)
# We want x and y to be in the range [–1000, 0] (i.e. 0 is the maximum, –1000 is the minimum)
workspace_bounds = {
    "x": (0, -1000),   # When mapped, x=0 => 0 and x=image_width => -1000
    "y": (0, -1000),
    "z": (0, 6000)
}

# Paintbrush width (used as spacing for strokes)
BRUSH_WIDTH = 10

##############################
#    Helper Functions        #
##############################
def load_image(image_path):
    """Loads an image from a given path."""
    img = cv2.imread(image_path)
    if img is None:
        print("Error: Could not load image.")
        return None
    print(f"Image loaded: {image_path}")
    return img

def preprocess_image(img):
    """Converts the image to grayscale and applies Gaussian blur."""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    return blurred

def detect_edges(blurred_img):
    """Applies Canny edge detection."""
    edges = cv2.Canny(blurred_img, 10, 20)
    cv2.imwrite(edgesImage, edges)
    return edges

def apply_morphological_operations(edges):
    """Applies morphological closing to fill gaps in the edges."""
    kernel = np.ones((5, 5), np.uint8)
    closed_edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
    cv2.imwrite(closedEdgesImage, closed_edges)
    return closed_edges

def find_contours(closed_edges):
    """Finds contours in the image."""
    contours, _ = cv2.findContours(closed_edges.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    print(f"Found {len(contours)} contours.")
    return contours

def approximate_contours(contours, epsilon_factor=0.01, min_area=500):
    """Approximates contours and identifies shapes."""
    shape_counts = {
        "Triangle": 0,
        "Rectangle": 0,
        "Square": 0,
        "Pentagon": 0,
        "Hexagon": 0,
        "Heptagon": 0,
        "Polygon": 0,
        "Circle": 0
    }
    shapes = []
    for contour in contours:
        contour_area = cv2.contourArea(contour)
        if contour_area < min_area:
            continue
        epsilon = epsilon_factor * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        shape_name = identify_shape(approx, contour_area)
        shape_counts[shape_name] += 1
        shapes.append((approx, shape_name))
    return shapes, shape_counts

def identify_shape(approx, contour_area):
    """Identifies the shape based on the number of vertices."""
    if len(approx) == 3:
        return "Triangle"
    elif len(approx) == 4:
        x, y, w, h = cv2.boundingRect(approx)
        aspect_ratio = float(w) / h
        return "Square" if 0.95 <= aspect_ratio <= 1.05 else "Rectangle"
    elif len(approx) == 5:
        return "Pentagon"
    elif len(approx) == 6:
        return "Hexagon"
    elif len(approx) == 7:
        return "Heptagon"
    elif len(approx) > 7:
        center, radius = cv2.minEnclosingCircle(approx)
        circle_area = np.pi * (radius ** 2)
        return "Circle" if abs(contour_area - circle_area) / circle_area < 0.2 else "Polygon"
    else:
        center, radius = cv2.minEnclosingCircle(approx)
        circle_area = np.pi * (radius ** 2)
        return "Circle" if abs(contour_area - circle_area) / circle_area < 0.2 else "Polygon"

#########################################
#  Stroke Generation (Unchanged Logic)  #
#########################################
def generate_strokes(approx, shape_name, brush_width):
    """
    Generate strokes for the given shape:
      - For a CIRCLE, create concentric arcs.
      - Otherwise (polygon), fill with line strokes parallel to its longest side.
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
    For a polygon (non-circle), fill it with line strokes that run parallel to its longest side.
      1. Find the longest edge and its angle.
      2. Rotate the polygon so that the longest edge is horizontal.
      3. Generate horizontal scanlines (in the rotated space) at intervals of brush_width.
      4. Find intersections with the polygon edges.
      5. Rotate the line endpoints back.
    Stroke format: "line, x1, y1, x2, y2, brushWidth"
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
    # Rotate polygon so longest edge is horizontal
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
    # Generate scanlines in rotated space
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

###################################################
#  Scaling Functions (Map to Workspace Bounds)    #
###################################################
def scale_point(pt, image_dims, workspace_bounds):
    """
    Maps a point (x, y) from image coordinates to workspace coordinates.
    The mapping is linear such that:
      - x=0 maps to workspace x = (0, as defined) and x=image_width maps to -1000.
      - Similarly for y.
    """
    width, height = image_dims
    # For x: we want a mapping from [0, width] to [0, -1000]
    # workspace_bounds["x"] = (0, -1000): min = -1000, max = 0.
    ws_x_min, ws_x_max = min(workspace_bounds["x"]), max(workspace_bounds["x"])
    ws_y_min, ws_y_max = min(workspace_bounds["y"]), max(workspace_bounds["y"])
    x, y = pt
    scaled_x = ws_x_min + (x / width) * (ws_x_max - ws_x_min)
    scaled_y = ws_y_min + (y / height) * (ws_y_max - ws_y_min)
    # This produces values between -1000 and 0.
    return (scaled_x, scaled_y)

def scale_stroke(stroke, image_dims, workspace_bounds):
    """
    Given a stroke definition string (line or arc), scale its coordinates to the workspace.
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
        # Use x-scale for radius (assuming uniform scaling)
        ws_x_min, ws_x_max = min(workspace_bounds["x"]), max(workspace_bounds["x"])
        scale_x = (ws_x_max - ws_x_min) / image_dims[0]
        scaled_r = r * scale_x
        return f"arc, {int(round(pt_center[0]))}, {int(round(pt_center[1]))}, {int(round(scaled_r))}, {start_angle}, {end_angle}, {brush}"
    else:
        return stroke

###################################################
#  Drawing Functions (Outlines + Workspace Strokes)  #
###################################################
def draw_contours_and_shapes(img, contours, shapes, shape_counts):
    """Draw contours, shape names, and fill shapes on the main images."""
    img_colored = np.zeros_like(img)
    contour_img = np.zeros_like(img)
    img_contours = img.copy()
    for approx, shape_name in shapes:
        strokes = generate_strokes(approx, shape_name, BRUSH_WIDTH)
        for stroke in strokes:
            print(f"Generated Stroke: {stroke}")
        # Draw the contour outlines in green
        cv2.drawContours(img_contours, [approx], -1, (0, 255, 0), 2)
        mask = np.zeros(img.shape[:2], dtype=np.uint8)
        cv2.drawContours(mask, [approx], -1, 255, thickness=cv2.FILLED)
        mean_color = cv2.mean(img, mask=mask)[:3]
        cv2.drawContours(img_colored, [approx], -1, mean_color, thickness=cv2.FILLED)
        x_text, y_text = approx.ravel()[0], approx.ravel()[1] - 10
        cv2.putText(img_contours, shape_name, (x_text, y_text),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        color = [random.randint(0, 255) for _ in range(3)]
        cv2.drawContours(contour_img, [approx], -1, color, 1)
    return img_colored, img_contours, contour_img

def draw_thin_strokes_image(img, shapes):
    """
    Create an image that overlays thin stroke lines and contour outlines on the original.
    The contour outlines are drawn in green and the strokes in white.
    """
    thin_image = img.copy()
    # Draw contour outlines in green
    for approx, shape_name in shapes:
        cv2.drawContours(thin_image, [approx], -1, (0, 255, 0), 1)
    # Overlay strokes in thin white
    for approx, shape_name in shapes:
        strokes = generate_strokes(approx, shape_name, BRUSH_WIDTH)
        for stroke_def in strokes:
            parts = [p.strip() for p in stroke_def.split(',')]
            cmd = parts[0].lower()
            if cmd == "line":
                _, x1, y1, x2, y2, _ = parts
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                cv2.line(thin_image, (x1, y1), (x2, y2), (255,255,255), 1)
            elif cmd == "arc":
                _, cx, cy, r, start_angle, end_angle, _ = parts
                cx, cy, r = int(cx), int(cy), int(r)
                cv2.ellipse(thin_image, (cx, cy), (r, r), 0.0,
                            float(start_angle), float(end_angle), (255,255,255), 1)
    cv2.imwrite(thinStrokesImage, thin_image)
    print(f"Thinner strokes image saved as {thinStrokesImage}")

def draw_workspace_strokes_image(all_strokes, image_dims, workspace_bounds):
    """
    Create a workspace image by scaling all strokes.
    
    (a) For debugging, print the scaled stroke definitions—they should have x,y values
        in the range [–1000, 0].
    (b) For visualization, we shift the coordinates to positive values to draw on a canvas.
    """
    ws_x_min, ws_x_max = min(workspace_bounds["x"]), max(workspace_bounds["x"])
    ws_y_min, ws_y_max = min(workspace_bounds["y"]), max(workspace_bounds["y"])
    
    # (a) Compute and print scaled stroke definitions
    print("Scaled stroke definitions (should have x,y between -1000 and 0):")
    scaled_strokes_list = []
    for stroke in all_strokes:
        scaled = scale_stroke(stroke, image_dims, workspace_bounds)
        print(scaled)
        scaled_strokes_list.append(scaled)
    
    # (b) For drawing, we must shift them to nonnegative coordinates.
    canvas_width = int(round(ws_x_max - ws_x_min))  # 0 - (-1000) = 1000
    canvas_height = int(round(ws_y_max - ws_y_min))
    canvas = np.zeros((canvas_height, canvas_width, 3), dtype=np.uint8)
    
    for scaled_stroke in scaled_strokes_list:
        parts = [p.strip() for p in scaled_stroke.split(',')]
        cmd = parts[0].lower()
        if cmd == "line":
            x1 = int(parts[1])
            y1 = int(parts[2])
            x2 = int(parts[3])
            y2 = int(parts[4])
            # Shift coordinates by adding the absolute value of ws_x_min and ws_y_min.
            x1_shift = x1 - ws_x_min
            y1_shift = y1 - ws_y_min
            x2_shift = x2 - ws_x_min
            y2_shift = y2 - ws_y_min
            cv2.line(canvas, (x1_shift, y1_shift), (x2_shift, y2_shift), (255,255,255), 1)
        elif cmd == "arc":
            cx = int(parts[1])
            cy = int(parts[2])
            r = int(parts[3])
            start_angle = float(parts[4])
            end_angle = float(parts[5])
            cx_shift = cx - ws_x_min
            cy_shift = cy - ws_y_min
            cv2.ellipse(canvas, (cx_shift, cy_shift), (r, r), 0.0, start_angle, end_angle, (255,255,255), 1)
    cv2.imwrite("workspace_strokes.png", canvas)
    print("Workspace strokes image saved as workspace_strokes.png")

def save_images(img_colored, img_contours, contour_img):
    """Saves processed images to disk."""
    cv2.imwrite(isolatedContours, contour_img)
    cv2.imwrite(shapesOnImage, img_contours)
    cv2.imwrite(colouredShapes, img_colored)

def print_shape_counts(shape_counts):
    """Prints counts of each detected shape."""
    print("\nShape Counts:")
    for shape, count in shape_counts.items():
        print(f"{shape}: {count}")

##################################
#          Main Function         #
##################################
def detect_shapes(image_path, epsilon_factor=0.01, min_area=500):
    img = load_image(image_path)
    if img is None:
        return
    image_dims = (img.shape[1], img.shape[0])  # (width, height)
    
    blurred_img = preprocess_image(img)
    edges = detect_edges(blurred_img)
    closed_edges = apply_morphological_operations(edges)
    contours = find_contours(closed_edges)
    
    shapes, shape_counts = approximate_contours(contours, epsilon_factor, min_area)
    img_colored, img_contours, contour_img = draw_contours_and_shapes(img, contours, shapes, shape_counts)
    
    save_images(img_colored, img_contours, contour_img)
    print_shape_counts(shape_counts)
    draw_thin_strokes_image(img, shapes)
    
    # Gather all stroke definitions from all shapes
    all_strokes = []
    for approx, shape_name in shapes:
        strokes = generate_strokes(approx, shape_name, BRUSH_WIDTH)
        all_strokes.extend(strokes)
    
    # Draw the scaled strokes onto a workspace canvas
    draw_workspace_strokes_image(all_strokes, image_dims, workspace_bounds)

if __name__ == "__main__":
    detect_shapes(inputImage, epsilon_factor=0.01, min_area=100)
