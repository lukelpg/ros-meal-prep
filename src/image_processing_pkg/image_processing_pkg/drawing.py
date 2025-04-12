# image_processing_pkg/drawing.py

import cv2
import numpy as np
import random
from image_processing_pkg.scaling import scale_stroke
from image_processing_pkg.stroke_mapping import generate_strokes
from image_processing_pkg.config import isolatedContours, shapesOnImage, colouredShapes

def draw_contours_and_shapes(img, contours, shapes, shape_counts):
    """Draws contours, shape names, and fills shapes on separate images."""
    img_colored = np.zeros_like(img)
    contour_img = np.zeros_like(img)
    img_contours = img.copy()
    for approx, shape_name in shapes:
        # Draw contour outlines in green
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

def draw_thin_strokes_image(img, shapes, brush_width=10, output_path="thinStrokes.png"):
    """
    Creates an image that overlays thin stroke lines and contour outlines on the original image.
    """
    thin_image = img.copy()
    # Draw contour outlines in green
    for approx, shape_name in shapes:
        cv2.drawContours(thin_image, [approx], -1, (0, 255, 0), 1)
    # Overlay strokes in white
    for approx, shape_name in shapes:
        strokes = generate_strokes(approx, shape_name, brush_width)
        for stroke_def in strokes:
            parts = [p.strip() for p in stroke_def.split(',')]
            cmd = parts[0].lower()
            if cmd == "line":
                _, x1, y1, x2, y2, _ = parts
                cv2.line(thin_image, (int(x1), int(y1)), (int(x2), int(y2)), (255,255,255), 1)
            elif cmd == "arc":
                _, cx, cy, r, start_angle, end_angle, _ = parts
                cv2.ellipse(thin_image, (int(cx), int(cy)), (int(r), int(r)), 0.0,
                            float(start_angle), float(end_angle), (255,255,255), 1)
    cv2.imwrite(output_path, thin_image)
    print(f"Thinner strokes image saved as {output_path}")


def draw_workspace_strokes_image(all_strokes, image_dims, workspace_bounds, output_path="workspace_strokes.png"):
    """
    Creates a workspace canvas by scaling all strokes to the defined workspace bounds.
    """
    ws_x_min, ws_x_max = min(workspace_bounds["x"]), max(workspace_bounds["x"])
    ws_y_min, ws_y_max = min(workspace_bounds["y"]), max(workspace_bounds["y"])
    
    print("Scaled stroke definitions (x,y should be between -1000 and 0):")
    scaled_strokes_list = []
    for stroke_tuple in all_strokes:
        # Unpack the tuple: stroke_string and shape_color (we ignore shape_color here)
        stroke_str, _ = stroke_tuple
        # Pass only the stroke string to scale_stroke
        scaled = scale_stroke(stroke_str, image_dims, workspace_bounds)
        print(scaled)
        scaled_strokes_list.append(scaled)
    
    canvas_width = int(round(ws_x_max - ws_x_min))
    canvas_height = int(round(ws_y_max - ws_y_min))
    canvas = np.zeros((canvas_height, canvas_width, 3), dtype=np.uint8)
    
    for scaled_stroke in scaled_strokes_list:
        parts = [p.strip() for p in scaled_stroke.split(',')]
        cmd = parts[0].lower()
        if cmd == "line":
            x1, y1, x2, y2 = int(parts[1]), int(parts[2]), int(parts[3]), int(parts[4])
            # Shift coordinates so they become positive on the canvas
            x1_shift = x1 - ws_x_min
            y1_shift = y1 - ws_y_min
            x2_shift = x2 - ws_x_min
            y2_shift = y2 - ws_y_min
            cv2.line(canvas, (x1_shift, y1_shift), (x2_shift, y2_shift), (255,255,255), 1)
        elif cmd == "arc":
            cx, cy, r = int(parts[1]), int(parts[2]), int(parts[3])
            start_angle = float(parts[4])
            end_angle = float(parts[5])
            cx_shift = cx - ws_x_min
            cy_shift = cy - ws_y_min
            cv2.ellipse(canvas, (cx_shift, cy_shift), (r, r), 0.0, start_angle, end_angle, (255,255,255), 1)
    cv2.imwrite(output_path, canvas)
    print(f"Workspace strokes image saved as {output_path}")

def save_images(img_colored, img_contours, contour_img):
    cv2.imwrite(isolatedContours, contour_img)
    cv2.imwrite(shapesOnImage, img_contours)
    cv2.imwrite(colouredShapes, img_colored)

def print_shape_counts(shape_counts):
    print("\nShape Counts:")
    for shape, count in shape_counts.items():
        print(f"{shape}: {count}")
