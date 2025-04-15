# image_processing_pkg/image_processor.py

import cv2
import numpy as np
import math
from image_processing_pkg.config import (
    inputImage, 
    edgesImage, 
    closedEdgesImage, 
    isolatedContours, 
    shapesOnImage, 
    colouredShapes, 
    thinStrokesImage,
    workspace_bounds,
    BRUSH_WIDTH,
    palette,
    DIP_Z,
    SAFE_Z
)
from image_processing_pkg.stroke_mapping import generate_strokes
from image_processing_pkg.drawing import (
    draw_contours_and_shapes, 
    draw_thin_strokes_image, 
    draw_workspace_strokes_image, 
    save_images, 
    print_shape_counts
)
from image_processing_pkg.color_utils import get_closest_palette_color

def load_image(image_path):
    """Loads an image from a given path."""
    img = cv2.imread(image_path)
    if img is None:
        print("Error: Could not load image.")
    else:
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

def detect_shapes(image_path, epsilon_factor=0.01, min_area=500):
    """
    Detect shapes in the image and generate stroke definitions.
    Each stroke is returned as a tuple (stroke_definition, shape_color) where shape_color is in RGB.
    A new "dip" stroke is inserted at the start of every shape.
    """
    img = load_image(image_path)
    if img is None:
        return [], (0, 0)
    image_dims = (img.shape[1], img.shape[0])  # (width, height)
    
    blurred_img = preprocess_image(img)
    edges = detect_edges(blurred_img)
    closed_edges = apply_morphological_operations(edges)
    contours = find_contours(closed_edges)
    
    shapes, shape_counts = approximate_contours(contours, epsilon_factor, min_area)
    img_colored, img_contours, contour_img = draw_contours_and_shapes(img, contours, shapes, shape_counts)
    
    save_images(img_colored, img_contours, contour_img)
    print_shape_counts(shape_counts)
    draw_thin_strokes_image(img, shapes, BRUSH_WIDTH, thinStrokesImage)
    
    # Generate stroke definitions along with the mean color for each shape.
    # For each shape, first insert a dip stroke command, then the shape’s drawing strokes.
    all_strokes = []
    for approx, shape_name in shapes:
        # Compute the mean color for this shape using a mask.
        mask = np.zeros(img.shape[:2], dtype=np.uint8)
        cv2.drawContours(mask, [approx], -1, 255, thickness=cv2.FILLED)
        # cv2.mean returns (B, G, R, A); convert to RGB integers.
        mean_color_bgr = cv2.mean(img, mask=mask)[:3]
        shape_color = (int(mean_color_bgr[2]), int(mean_color_bgr[1]), int(mean_color_bgr[0]))
        
        # Determine the palette color for this shape.
        required_color_key = get_closest_palette_color(shape_color)
        color_hex = palette[required_color_key]["hex"]
        pickup_coords = palette[required_color_key]["coord"]  # (x, y)
        
        # Create a dip stroke command.
        # Command format:
        #   dip, pickup_x, pickup_y, DIP_Z, safe_x, safe_y, SAFE_Z, color_hex
        # We assume that the safe coordinate is the same as the pickup coordinate.
        dip_command = f"dip, {pickup_coords[0]}, {pickup_coords[1]}, {DIP_Z}, {pickup_coords[0]}, {pickup_coords[1]}, {SAFE_Z}, {color_hex}"
        
        # Insert the dip stroke at the start for this shape.
        shape_strokes = []
        shape_strokes.append((dip_command, shape_color))
        
        # Generate the normal stroke commands for this shape (line/arc).
        strokes = generate_strokes(approx, shape_name, BRUSH_WIDTH)
        for stroke in strokes:
            shape_strokes.append((stroke, shape_color))
        
        # Append all strokes for this shape.
        all_strokes.extend(shape_strokes)
    
    draw_workspace_strokes_image(all_strokes, image_dims, workspace_bounds, "workspace_strokes.png")
    return all_strokes, image_dims

if __name__ == "__main__":
    # For standalone testing
    strokes, dims = detect_shapes(inputImage, epsilon_factor=0.1, min_area=100)
    print("Generated strokes:")
    for stroke, color in strokes:
        print(stroke, color)
