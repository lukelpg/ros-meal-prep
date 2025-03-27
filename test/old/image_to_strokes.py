import cv2
import numpy as np

class ImageToStrokesVisualizer:
    def __init__(self, image_path="red_x.png", target_size=500, debug_size=600):
        self.image_path = image_path
        self.target_size = target_size  # Maps image to (-500, -500)
        self.debug_size = debug_size  # Fixed debug window size

    def load_image(self):
        """Loads image and applies edge detection."""
        image = cv2.imread(self.image_path, cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise FileNotFoundError(f"Could not load image: {self.image_path}")

        self.image_height, self.image_width = image.shape[:2]

        edges = cv2.Canny(image, 80, 180)  # Simple edge detection
        cv2.imshow("Simplified Edges", edges)
        return edges

    def detect_lines(self, edges):
        """Detects lines using Hough Transform."""
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 100, minLineLength=50, maxLineGap=10)
        if lines is None:
            print("\n[INFO] No lines detected in the image.")
            return []
        return lines

    def map_coordinates(self, x, y):
        """Maps image coordinates to robot workspace (-500, -500)."""
        x_mapped = -((x / self.image_width) * self.target_size)  
        y_mapped = -((y / self.image_height) * self.target_size)  
        return int(x_mapped), int(y_mapped)

    def normalize_for_debug(self, x, y):
        """Centers strokes inside the debug window."""
        x_debug = int(((x + self.target_size / 2) / self.target_size) * self.debug_size)
        y_debug = int(((y + self.target_size / 2) / self.target_size) * self.debug_size)
        return x_debug, y_debug

    def bezier_curve(self, p0, p1, p2, num_points=10):
        """Generates a Quadratic Bezier Curve."""
        t_vals = np.linspace(0, 1, num_points)
        return [
            (
                int((1 - t) ** 2 * p0[0] + 2 * (1 - t) * t * p1[0] + t ** 2 * p2[0]),
                int((1 - t) ** 2 * p0[1] + 2 * (1 - t) * t * p1[1] + t ** 2 * p2[1])
            )
            for t in t_vals
        ]

    def extract_strokes(self, lines):
        """Converts detected lines into accurate stroke commands."""
        stroke_commands = []

        # ✅ Fix: Properly check if `lines` is empty
        if lines is not None and len(lines) > 0:
            print("\n[INFO] Generated Strokes:")
            for line in lines:
                x1, y1, x2, y2 = line[0]

                x1, y1 = self.map_coordinates(x1, y1)
                x2, y2 = self.map_coordinates(x2, y2)

                length = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

                if length > 100:  
                    mid_x, mid_y = (x1 + x2) // 2, (y1 + y2) // 2
                    control_x, control_y = mid_x, mid_y - 30  
                    bezier_points = self.bezier_curve((x1, y1), (control_x, control_y), (x2, y2))

                    for i in range(len(bezier_points) - 1):
                        x_start, y_start = bezier_points[i]
                        x_end, y_end = bezier_points[i + 1]
                        stroke = f"line, {x_start}, {y_start}, {x_end}, {y_end}, 100"
                        stroke_commands.append(stroke)
                        print(stroke)
                else:
                    stroke = f"line, {x1}, {y1}, {x2}, {y2}, 100"
                    stroke_commands.append(stroke)
                    print(stroke)
        else:
            print("\n[INFO] No strokes were generated. Check image quality.")

        with open("strokes.txt", "w") as f:
            for stroke in stroke_commands:
                f.write(stroke + "\n")

        return stroke_commands


    def draw_strokes(self, lines):
        """Creates a blank canvas and overlays the detected strokes."""
        stroke_canvas = np.ones((self.debug_size, self.debug_size, 3), dtype=np.uint8) * 255  # White canvas

        # ✅ Fix: Properly check if `lines` is empty
        if lines is not None and len(lines) > 0:
            for line in lines:
                x1, y1, x2, y2 = line[0]

                x1, y1 = self.map_coordinates(x1, y1)
                x2, y2 = self.map_coordinates(x2, y2)

                x1_disp, y1_disp = self.normalize_for_debug(x1, y1)
                x2_disp, y2_disp = self.normalize_for_debug(x2, y2)

                if np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2) > 100:
                    control_x, control_y = (x1 + x2) // 2, (y1 + y2) // 2 - 30
                    bezier_points = self.bezier_curve((x1_disp, y1_disp), (control_x, control_y), (x2_disp, y2_disp))

                    for i in range(len(bezier_points) - 1):
                        cv2.line(stroke_canvas, bezier_points[i], bezier_points[i + 1], (0, 255, 0), 2)
                else:
                    cv2.line(stroke_canvas, (x1_disp, y1_disp), (x2_disp, y2_disp), (0, 0, 255), 2)
        else:
            print("\n[INFO] No strokes to draw. Skipping visualization.")

        return stroke_canvas


    def process_image(self):
        """Processes the image and displays detected edges and strokes."""
        edges = self.load_image()
        lines = self.detect_lines(edges)
        self.extract_strokes(lines)
        strokes_image = self.draw_strokes(lines)

        cv2.imshow("Detected Strokes (Centered)", strokes_image)
        print("\n[INFO] Press 'q' to close the windows.")

        while True:
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break  

        cv2.destroyAllWindows()

if __name__ == "__main__":
    visualizer = ImageToStrokesVisualizer("red_x.png")  
    visualizer.process_image()
