import cv2
import numpy as np

# Function to detect shapes in an image
def detect_shapes(image_path, output_path="shapes_output.png"):
    # Load the image
    img = cv2.imread(image_path)
    
    # Convert to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # Apply GaussianBlur to reduce noise and improve contour detection
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Perform edge detection using Canny
    edges = cv2.Canny(blurred, 50, 150)
    
    # Find contours from the edges
    contours, _ = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Create a copy of the original image to draw contours on
    img_contours = img.copy()

    # Iterate through contours and draw shapes
    for contour in contours:
        # Approximate the contour to a polygon
        epsilon = 0.04 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)

        # Draw the contour
        if len(approx) == 3:
            shape_name = "Triangle"
        elif len(approx) == 4:
            # Check if it's a square or rectangle
            x, y, w, h = cv2.boundingRect(approx)
            aspect_ratio = float(w) / h
            shape_name = "Square" if 0.95 <= aspect_ratio <= 1.05 else "Rectangle"
        elif len(approx) > 4:
            shape_name = "Circle"
        else:
            shape_name = "Other"

        # Draw the shape and label
        cv2.drawContours(img_contours, [approx], -1, (0, 255, 0), 2)
        x, y = approx.ravel()[0], approx.ravel()[1] - 10
        cv2.putText(img_contours, shape_name, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    # Save the output image
    cv2.imwrite(output_path, img_contours)
    print(f"Shapes detected and saved to {output_path}")

# Example usage
image_path = "car.png"  # Replace with the path to your image
detect_shapes(image_path)
