# image_processing_pkg/config.py

# File names for intermediate and output images
location="/home/luke/Projects/robots/ros-meal-prep/src/image_processing_pkg/image_processing_pkg/images/"
inputImage = f"{location}red_x.png"
edgesImage = f"{location}roughEdges.png"
closedEdgesImage = f"{location}closedEdges.png"
isolatedContours = f"{location}isolatedContours.png"
shapesOnImage = f"{location}shapesOnImage.png"
colouredShapes = f"{location}colouredShapes.png"
thinStrokesImage = f"{location}thinStrokes.png"

# Workspace bounds for x, y (z is ignored for stroke mapping)
workspace_bounds = {
    "x": (0, -1000),  # Maps image x: 0 -> 0 and image x: width -> -1000
    "y": (0, -1000),
    "z": (0, 6000)
}

# Brush width used for stroke spacing
BRUSH_WIDTH = 45
