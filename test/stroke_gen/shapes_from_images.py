import cv2
import numpy as np
import random
import math

inputImage = "linux.png"
edgesImage = "roughEdges.png"
closedEdgesImage = "closedEdges.png"
isolatedContours = "isolatedContours.png"
shapesOnImage = "shapesOnImage.png"
colouredShapes = "colouredShapes.png"
thinStrokesImage = "thinStrokes.png"

# Paintbrush width (constant)
BRUSH_WIDTH = 10  # Spacing for lines or arcs

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
        if contour_area < min_area:  # Skip small contours
            continue
        
        epsilon = epsilon_factor * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        
        shape_name = identify_shape(approx, contour_area)
        shape_counts[shape_name] += 1
        
        # Store both the approx polygon and the identified shape
        shapes.append((approx, shape_name))
    
    return shapes, shape_counts

def identify_shape(approx, contour_area):
    """Identifies the shape based on the number of vertices in the contour approximation."""
    if len(approx) == 3:
        return "Triangle"
    elif len(approx) == 4:
        x, y, w, h = cv2.boundingRect(approx)
        aspect_ratio = float(w) / h
        if 0.95 <= aspect_ratio <= 1.05:
            return "Square"
        else:
            return "Rectangle"
    elif len(approx) == 5:
        return "Pentagon"
    elif len(approx) == 6:
        return "Hexagon"
    elif len(approx) == 7:
        return "Heptagon"
    elif len(approx) > 7:
        # Double-check if it's circle-like by area
        center, radius = cv2.minEnclosingCircle(approx)
        circle_area = np.pi * (radius ** 2)
        if abs(contour_area - circle_area) / circle_area < 0.2:
            return "Circle"
        else:
            return "Polygon"
    else:
        # Fallback check for circle
        center, radius = cv2.minEnclosingCircle(approx)
        circle_area = np.pi * (radius ** 2)
        if abs(contour_area - circle_area) / circle_area < 0.2:
            return "Circle"
        else:
            return "Polygon"

###############################################################################
#                          STROKE GENERATION LOGIC                            #
###############################################################################
def generate_strokes(approx, shape_name, brush_width):
    """
    Generate strokes for the given shape:
      - If it's a CIRCLE, create concentric arcs (rings).
      - Otherwise (polygon), create back-and-forth line strokes inside the contour
        that are parallel to the longest side.
    
    Returns a list of stroke definitions (strings).
    """
    if shape_name == "Circle":
        return generate_circle_arcs(approx, brush_width)
    else:
        return generate_polygon_lines(approx, brush_width)

def generate_circle_arcs(approx, brush_width):
    """
    For a circle contour, we fill it with concentric arcs (i.e. rings).
    Each arc: "arc, cx, cy, r, startAngle, endAngle, brushWidth"
    We'll step radius from 0 up to the circle's radius in increments of brush_width.
    """
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
    For a polygon (non-circle), fill it with line strokes that run parallel to its
    longest side. The algorithm is as follows:
      1. Determine the longest edge and its angle.
      2. Rotate the polygon so that this edge is horizontal.
      3. Generate horizontal scanlines (in the rotated space) at intervals of brush_width.
      4. Compute intersections of each scanline with the rotated polygon edges.
      5. Transform the line endpoints back to the original coordinate system.
      
    Stroke format: "line, x1, y1, x2, y2, brushWidth"
    """
    pts = approx.reshape(-1, 2)
    
    # Find the longest edge and compute its angle
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
    
    # Rotation matrix to rotate by -longest_angle
    cos_theta = math.cos(longest_angle)
    sin_theta = math.sin(longest_angle)
    R = np.array([[cos_theta, sin_theta],
                  [-sin_theta, cos_theta]])
    
    # Rotate all points
    pts_rotated = np.dot(pts, R.T)
    
    # Determine bounding box in rotated coordinates
    min_y = np.min(pts_rotated[:,1])
    max_y = np.max(pts_rotated[:,1])
    
    strokes = []
    start_y = int(math.floor(min_y))
    end_y = int(math.ceil(max_y))
    
    # For each scanline in rotated coordinates (parallel to the longest edge)
    for scan_y in range(start_y, end_y+1, brush_width):
        intersections = []
        # Loop over edges in rotated coordinates
        for i in range(n):
            p1 = pts_rotated[i]
            p2 = pts_rotated[(i+1) % n]
            # Check if the scanline y=scan_y intersects the edge
            if (p1[1] <= scan_y and p2[1] > scan_y) or (p2[1] <= scan_y and p1[1] > scan_y):
                if abs(p2[1] - p1[1]) < 1e-6:
                    continue
                t = (scan_y - p1[1]) / (p2[1] - p1[1])
                intersect_x = p1[0] + t * (p2[0] - p1[0])
                intersections.append(intersect_x)
        
        intersections.sort()
        
        # Pair up intersections to form line segments
        for j in range(0, len(intersections)-1, 2):
            x_left = intersections[j]
            x_right = intersections[j+1]
            # Define endpoints in rotated coordinates
            pt1_rot = np.array([x_left, scan_y])
            pt2_rot = np.array([x_right, scan_y])
            
            # Inverse rotation matrix (rotation by longest_angle)
            R_inv = np.array([[cos_theta, -sin_theta],
                              [sin_theta, cos_theta]])
            pt1_orig = np.dot(R_inv, pt1_rot)
            pt2_orig = np.dot(R_inv, pt2_rot)
            pt1 = (int(round(pt1_orig[0])), int(round(pt1_orig[1])))
            pt2 = (int(round(pt2_orig[0])), int(round(pt2_orig[1])))
            stroke = f"line, {pt1[0]}, {pt1[1]}, {pt2[0]}, {pt2[1]}, {brush_width}"
            strokes.append(stroke)
    
    return strokes

###############################################################################
#                DRAWING THE SHAPES, CONTOURS, AND STROKES                   #
###############################################################################
def draw_contours_and_shapes(img, contours, shapes, shape_counts):
    """Draw contours, shape names, and fill contours with colors on the main images."""
    img_colored = np.zeros_like(img)
    contour_img = np.zeros_like(img)
    img_contours = img.copy()
    
    for approx, shape_name in shapes:
        # Generate the strokes for this shape
        strokes = generate_strokes(approx, shape_name, BRUSH_WIDTH)
        for stroke in strokes:
            print(f"Generated Stroke: {stroke}")
        
        # Draw the contour in green
        cv2.drawContours(img_contours, [approx], -1, (0, 255, 0), 2)

        # Fill shape with its average color
        mask = np.zeros(img.shape[:2], dtype=np.uint8)
        cv2.drawContours(mask, [approx], -1, 255, thickness=cv2.FILLED)
        mean_color = cv2.mean(img, mask=mask)[:3]
        cv2.drawContours(img_colored, [approx], -1, mean_color, thickness=cv2.FILLED)
        
        # Label the shape
        x_text, y_text = approx.ravel()[0], approx.ravel()[1] - 10
        cv2.putText(img_contours, shape_name, (x_text, y_text),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        
        # Draw the contour on a blank image with random color
        color = [random.randint(0, 255) for _ in range(3)]
        cv2.drawContours(contour_img, [approx], -1, color, 1)

    return img_colored, img_contours, contour_img

def save_images(img_colored, img_contours, contour_img):
    """Saves the processed images to disk."""
    cv2.imwrite(isolatedContours, contour_img)
    cv2.imwrite(shapesOnImage, img_contours)
    cv2.imwrite(colouredShapes, img_colored)

def print_shape_counts(shape_counts):
    """Prints the counts of each shape detected."""
    print("\nShape Counts:")
    for shape, count in shape_counts.items():
        print(f"{shape}: {count}")

#####################################################################
#       Create a separate image with thinner stroke rendering       #
#####################################################################
def draw_thin_strokes_image(img, shapes):
    """
    Create a new image to visualize the strokes with thinner lines.
    This helps see the spacing of lines or arcs clearly.
    """
    thin_image = img.copy()
    
    # Draw the polygon/circle boundaries in thin green
    for approx, shape_name in shapes:
        cv2.drawContours(thin_image, [approx], -1, (0, 255, 0), 1)

    # Draw strokes in thin white
    for approx, shape_name in shapes:
        strokes = generate_strokes(approx, shape_name, BRUSH_WIDTH)
        
        for stroke_def in strokes:
            parts = stroke_def.split(',')
            cmd_type = parts[0].strip().lower()
            
            if cmd_type == "line":
                # line, x1, y1, x2, y2, brushW
                _, x1, y1, x2, y2, _ = parts
                x1, y1 = int(x1), int(y1)
                x2, y2 = int(x2), int(y2)
                cv2.line(thin_image, (x1, y1), (x2, y2), (255, 255, 255), 1)
            
            elif cmd_type == "arc":
                # arc, cx, cy, r, start, end, brushW
                _, cx, cy, r, start_angle, end_angle, _ = parts
                cx, cy = int(cx), int(cy)
                r = int(r)
                start_angle = float(start_angle)
                end_angle = float(end_angle)
                cv2.ellipse(
                    thin_image, (cx, cy), (r, r),
                    0.0, start_angle, end_angle,
                    (255, 255, 255), 1
                )
    
    cv2.imwrite(thinStrokesImage, thin_image)
    print(f"Thinner strokes image saved as {thinStrokesImage}")

###############################################################################
#                               MAIN FUNCTION                                 #
###############################################################################
def detect_shapes(image_path, epsilon_factor=0.01, min_area=500):
    """Main function to detect shapes in an image."""
    img = load_image(image_path)
    if img is None:
        return
    
    blurred_img = preprocess_image(img)
    edges = detect_edges(blurred_img)
    closed_edges = apply_morphological_operations(edges)
    contours = find_contours(closed_edges)
    
    shapes, shape_counts = approximate_contours(contours, epsilon_factor, min_area)
    img_colored, img_contours, contour_img = draw_contours_and_shapes(img, contours, shapes, shape_counts)
    
    save_images(img_colored, img_contours, contour_img)
    print_shape_counts(shape_counts)
    
    # Create a separate image to visualize the strokes in thin form
    draw_thin_strokes_image(img, shapes)

# Run shape detection
if __name__ == "__main__":
    detect_shapes(inputImage, epsilon_factor=0.01, min_area=100)
