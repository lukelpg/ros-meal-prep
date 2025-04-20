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
    BRUSH_WIDTH
)
from image_processing_pkg.stroke_mapping import generate_strokes
from image_processing_pkg.drawing import (
    draw_contours_and_shapes,
    draw_thin_strokes_image,
    draw_workspace_strokes_image,
    save_images,
    print_shape_counts
)

def load_image(image_path):
    img = cv2.imread(image_path)
    if img is None:
        print("Error: Could not load image.")
    else:
        print(f"Image loaded: {image_path}")
    return img

def preprocess_image(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return cv2.GaussianBlur(gray, (5, 5), 0)

def detect_edges(blurred):
    edges = cv2.Canny(blurred, 10, 20)
    cv2.imwrite(edgesImage, edges)
    return edges

def apply_morphological_operations(edges):
    kernel = np.ones((5,5), np.uint8)
    closed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
    cv2.imwrite(closedEdgesImage, closed)
    return closed

def find_contours(closed):
    contours, _ = cv2.findContours(closed.copy(),
                                  cv2.RETR_EXTERNAL,
                                  cv2.CHAIN_APPROX_SIMPLE)
    print(f"Found {len(contours)} contours.")
    return contours

def identify_shape(approx, area):
    if len(approx) == 3:
        return "Triangle"
    elif len(approx) == 4:
        x,y,w,h = cv2.boundingRect(approx)
        ar = w/float(h)
        return "Square" if 0.95<=ar<=1.05 else "Rectangle"
    elif len(approx) == 5:
        return "Pentagon"
    elif len(approx) == 6:
        return "Hexagon"
    elif len(approx) == 7:
        return "Heptagon"
    else:
        center,r = cv2.minEnclosingCircle(approx)
        circle_area = math.pi*(r*r)
        return "Circle" if abs(area-circle_area)/circle_area<0.2 else "Polygon"

def approximate_contours(contours, eps=0.01, min_area=500):
    counts = {s:0 for s in ["Triangle","Rectangle","Square","Pentagon","Hexagon","Heptagon","Polygon","Circle"]}
    shapes = []
    for c in contours:
        area = cv2.contourArea(c)
        if area<min_area: continue
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, eps*peri, True)
        name = identify_shape(approx, area)
        counts[name]+=1
        shapes.append((approx, name))
    return shapes, counts

def detect_shapes(image_path, epsilon_factor=0.01, min_area=500):
    img = load_image(image_path)
    if img is None:
        return [], (0,0)
    dims = (img.shape[1], img.shape[0])

    blurred = preprocess_image(img)
    edges = detect_edges(blurred)
    closed = apply_morphological_operations(edges)
    contours = find_contours(closed)

    shapes, shape_counts = approximate_contours(contours, epsilon_factor, min_area)
    img_col, img_cont, img_iso = draw_contours_and_shapes(img, contours, shapes, shape_counts)
    save_images(img_col, img_cont, img_iso)
    print_shape_counts(shape_counts)
    draw_thin_strokes_image(img, shapes, BRUSH_WIDTH, thinStrokesImage)

    # Build one group per shape
    shape_stroke_groups = []
    for approx, shape_name in shapes:
        # compute mean color
        mask = np.zeros(img.shape[:2], dtype=np.uint8)
        cv2.drawContours(mask, [approx], -1, 255, thickness=cv2.FILLED)
        mean_bgr = cv2.mean(img, mask=mask)[:3]
        shape_color = (int(mean_bgr[2]), int(mean_bgr[1]), int(mean_bgr[0]))

        strokes = generate_strokes(approx, shape_name, BRUSH_WIDTH)
        group = []
        for s in strokes:
            group.append((s, shape_color))
        shape_stroke_groups.append(group)

    # visualize all strokes if desired
    flat = [(s,c) for grp in shape_stroke_groups for s,c in grp]
    draw_workspace_strokes_image(flat, dims, workspace_bounds, "workspace_strokes.png")

    return shape_stroke_groups, dims

if __name__=="__main__":
    batches, dims = detect_shapes(inputImage, epsilon_factor=0.2, min_area=1000)
    for i,grp in enumerate(batches):
        print(f"Shape {i}:")
        for stroke,color in grp:
            print(" ", stroke, color)
