# image_processing_pkg/config.py

# File names for intermediate and output images
location="/home/luke/Projects/robots/ros-meal-prep/src/image_processing_pkg/image_processing_pkg/images/"
inputImage = f"{location}blue_x.png"
edgesImage = f"{location}roughEdges.png"
closedEdgesImage = f"{location}closedEdges.png"
isolatedContours = f"{location}isolatedContours.png"
shapesOnImage = f"{location}shapesOnImage.png"
colouredShapes = f"{location}colouredShapes.png"
thinStrokesImage = f"{location}thinStrokes.png"

# Workspace bounds for x, y (z is ignored for stroke mapping)
workspace_bounds = {
    "x": (0, -900),  # Maps image x: 0 -> 0 and image x: width -> -1000
    "y": (0, -900),
    "z": (0, 6000)
}

palette = {
    "red":    {"hex": "#DE0B0B", "coord": (-150, -1000)},
    "green":  {"hex": "#106E29", "coord": (-300, -1000)},
    "blue":   {"hex": "#0980DB", "coord": (-800, -1200)},
    "yellow": {"hex": "#FCCB08", "coord": (-600, -1000)},
    "black":   {"hex": "#000000", "coord": (-750, -1000)},
    "purple":{"hex": "#55068A", "coord": (-900, -1000)}
}

SAFE_Z = 2000
DIP_Z = 700 

# Brush width used for stroke spacing
BRUSH_WIDTH = 45
