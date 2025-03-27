import cv2
import numpy as np

testPath = "test.png"
testPath1 = "test1.png"

# Function to detect shapes in an image
def detect_shapes(image_path, output_path="shapes_output.png"):
    # Load the image
    img = cv2.imread(image_path)
    
    # Convert to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # Apply GaussianBlur to reduce noise and improve contour detection
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    cv2.imwrite(testPath, blurred)
    
    # Perform edge detection using Canny (make it more sensitive)
    edges = cv2.Canny(blurred, 10, 20)  # Lower the thresholds for more sensitivity
    cv2.imwrite(testPath1, edges)
    
    # Find contours from the edges
    contours, _ = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Create a copy of the original image to draw contours on
    img_contours = img.copy()

    # Iterate through contours and draw shapes
    for contour in contours:
        # Approximate the contour to a polygon (reduce epsilon to make it more sensitive)
        epsilon = 0.02 * cv2.arcLength(contour, True)  # Reduce epsilon to make approximation closer
        approx = cv2.approxPolyDP(contour, epsilon, True)

        # Draw the contour
        if len(approx) == 3:
            shape_name = "Triangle"
        elif len(approx) == 4:
            # Check if it's a square or rectangle
            x, y, w, h = cv2.boundingRect(approx)
            aspect_ratio = float(w) / h
            shape_name = "Square" if 0.95 <= aspect_ratio <= 1.05 else "Rectangle"
        elif len(approx) == 5:
            # Classify shapes with 5 vertices as Pentagons
            shape_name = "Pentagon"
        elif len(approx) == 6:
            # Classify shapes with 6 vertices as Hexagons
            shape_name = "Hexagon"
        elif len(approx) == 7:
            # Classify shapes with 7 vertices as Heptagons
            shape_name = "Heptagon"
        elif len(approx) > 7:
            # For shapes with more than 7 vertices, classify as Polygon
            shape_name = "Polygon"
        else:
            # Check for circles by comparing contour area and the area of the minimum enclosing circle
            center, radius = cv2.minEnclosingCircle(contour)
            circle_area = np.pi * (radius ** 2)
            contour_area = cv2.contourArea(contour)
            if abs(contour_area - circle_area) / circle_area < 0.2:
                shape_name = "Circle"
            else:
                shape_name = "Polygon"  # For other polygons that are not regular circles

        # Draw the shape and label
        cv2.drawContours(img_contours, [approx], -1, (0, 255, 0), 2)
        x, y = approx.ravel()[0], approx.ravel()[1] - 10
        cv2.putText(img_contours, shape_name, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    # Save the output image
    cv2.imwrite(output_path, img_contours)
    print(f"Shapes detected and saved to {output_path}")

# Example usage
image_path = "linux.png"  # Replace with the path to your image
detect_shapes(image_path)
