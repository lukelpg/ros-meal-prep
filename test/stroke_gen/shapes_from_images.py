import cv2 
import numpy as np

testPath = "test.png"
testPath1 = "test1.png"

def detect_shapes(image_path, output_path="shapes_output.png"):
    img = cv2.imread(image_path)
    if img is None:
        print("Error: Could not load image.")
        return
    print(f"Image loaded: {image_path}")

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    print("Converted to grayscale.")
    
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    cv2.imwrite(testPath, blurred)
    print(f"Gaussian blur applied. Image saved to {testPath}")
    
    edges = cv2.Canny(blurred, 10, 20)
    cv2.imwrite(testPath1, edges)
    print(f"Canny edge detection applied. Image saved to {testPath1}")
    
    contours, _ = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    print(f"Found {len(contours)} contours.")
    
    img_contours = img.copy()

    for i, contour in enumerate(contours):
        epsilon = 0.02 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        
        print(f"Contour {i+1}: Number of vertices in approximation: {len(approx)}")
        
        if len(approx) == 3:
            shape_name = "Triangle"
        elif len(approx) == 4:
            x, y, w, h = cv2.boundingRect(approx)
            aspect_ratio = float(w) / h
            shape_name = "Square" if 0.95 <= aspect_ratio <= 1.05 else "Rectangle"
        elif len(approx) == 5:
            shape_name = "Pentagon"
        elif len(approx) == 6:
            shape_name = "Hexagon"
        elif len(approx) == 7:
            shape_name = "Heptagon"
        elif len(approx) > 7:
            shape_name = "Polygon"
        else:
            center, radius = cv2.minEnclosingCircle(contour)
            circle_area = np.pi * (radius ** 2)
            contour_area = cv2.contourArea(contour)
            if abs(contour_area - circle_area) / circle_area < 0.2:
                shape_name = "Circle"
            else:
                shape_name = "Polygon"
        
        print(f"Contour {i+1}: Shape classified as {shape_name}")
        
        cv2.drawContours(img_contours, [approx], -1, (0, 255, 0), 2)
        x, y = approx.ravel()[0], approx.ravel()[1] - 10
        cv2.putText(img_contours, shape_name, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    cv2.imwrite(output_path, img_contours)
    print(f"Shapes detected and saved to {output_path}")

image_path = "linux.png"
detect_shapes(image_path)
