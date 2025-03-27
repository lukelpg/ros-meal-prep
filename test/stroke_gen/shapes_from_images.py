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

# Paintbrush width (constant)
BRUSH_WIDTH = 10  # Modify as needed

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
        
        if contour_area < min_area:  # Skip small contours based on min_area
            continue
        
        epsilon = epsilon_factor * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        
        shape_name = identify_shape(approx, contour_area)
        shape_counts[shape_name] += 1
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
        return "Polygon"
    else:
        # Handle circles based on contour area comparison
        center, radius = cv2.minEnclosingCircle(approx)
        circle_area = np.pi * (radius ** 2)
        if abs(contour_area - circle_area) / circle_area < 0.2:
            return "Circle"
        else:
            return "Polygon"

def generate_strokes(approx, brush_width):
    """Generate strokes (lines or arcs) to fill the contours based on the shape approximation."""
    strokes = []
    num_points = len(approx)
    
    for i in range(num_points):
        start = approx[i][0]
        end = approx[(i + 1) % num_points][0]  # Connect back to the first point
        
        # If the points form a straight line, generate line strokes
        if is_line(start, end):
            stroke = f"line, {start[0]}, {start[1]}, {end[0]}, {end[1]}, {brush_width}"
            strokes.append(stroke)
        
        # If the points form a curve (more than 2), generate arc strokes
        else:
            # Example: if we have a curve segment, generate arcs (for simplicity)
            center = ((start[0] + end[0]) // 2, (start[1] + end[1]) // 2)
            radius = int(math.hypot(end[0] - start[0], end[1] - start[1]) / 2)
            stroke = f"arc, {center[0]}, {center[1]}, {radius}, 0, 180, {brush_width}"
            strokes.append(stroke)
    
    return strokes

def is_line(start, end):
    """Determine if the segment is a straight line or not."""
    return abs(start[1] - end[1]) < BRUSH_WIDTH and abs(start[0] - end[0]) < BRUSH_WIDTH

def draw_contours_and_shapes(img, contours, shapes, shape_counts):
    """Draw contours, shape names, and fill contours with colors."""
    img_colored = np.zeros_like(img)
    contour_img = np.zeros_like(img)
    img_contours = img.copy()
    
    for approx, shape_name in shapes:
        # Generate strokes for the shape approximation
        strokes = generate_strokes(approx, BRUSH_WIDTH)
        for stroke in strokes:
            print(f"Generated Stroke: {stroke}")
        
        # Draw contours and fill with mean color
        cv2.drawContours(img_contours, [approx], -1, (0, 255, 0), 2)

        # Calculate the average color inside the contour
        mask = np.zeros_like(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
        cv2.drawContours(mask, [approx], -1, 255, thickness=cv2.FILLED)
        mean_color = cv2.mean(img, mask=mask)[:3]  # Get BGR mean color
        cv2.drawContours(img_colored, [approx], -1, mean_color, thickness=cv2.FILLED)
        
        x, y = approx.ravel()[0], approx.ravel()[1] - 10
        cv2.putText(img_contours, shape_name, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        
        # Draw individual contours on a blank image with random colors
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

detect_shapes(inputImage, epsilon_factor=0.02, min_area=100)
