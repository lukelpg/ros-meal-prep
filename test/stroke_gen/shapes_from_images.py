import cv2
import numpy as np
import random

inputImage = "linux.png"

edgesImage = "roughEdges.png"
closedEdgesImage = "closedEdges.png"
isolatedContours = "isolatedContours.png"
shapesOnImage = "shapesOnImage.png"
colouredShapes = "colouredShapes.png"


def detect_shapes(image_path):
    img = cv2.imread(image_path)
    if img is None:
        print("Error: Could not load image.")
        return
    print(f"Image loaded: {image_path}")

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    
    edges = cv2.Canny(blurred, 10, 20)
    cv2.imwrite(edgesImage, edges)
    
    # Apply morphological closing to fill gaps in the edges
    kernel = np.ones((5, 5), np.uint8)
    closed_edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
    cv2.imwrite(closedEdgesImage, closed_edges)

    # Use RETR_TREE to get all contours, not just the external ones
    contours, _ = cv2.findContours(closed_edges.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    print(f"Found {len(contours)} contours.")
    
    img_colored = np.zeros_like(img)
    contour_img = np.zeros_like(img)
    img_contours = img.copy()

    for i, contour in enumerate(contours):
        # Reduce epsilon for better approximation accuracy
        epsilon = 0.01 * cv2.arcLength(contour, True)
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

        # Calculate the average color inside the contour
        mask = np.zeros_like(gray)
        cv2.drawContours(mask, [contour], -1, 255, thickness=cv2.FILLED)
        mean_color = cv2.mean(img, mask=mask)[:3]  # Get BGR mean color

        # Fill the contour with the mean color
        cv2.drawContours(img_colored, [contour], -1, mean_color, thickness=cv2.FILLED)

        # Put the shape name text in the image
        x, y = approx.ravel()[0], approx.ravel()[1] - 10
        cv2.putText(img_colored, shape_name, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        cv2.putText(img_contours, shape_name, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        
        # Draw each individual contour on the blank image with a random color
        color = [random.randint(0, 255) for _ in range(3)]
        cv2.drawContours(contour_img, [contour], -1, color, 1)

    # Save images
    cv2.imwrite(isolatedContours, contour_img)
    cv2.imwrite(shapesOnImage, img_colored)
    cv2.imwrite(colouredShapes, img_contours)


detect_shapes(inputImage)
