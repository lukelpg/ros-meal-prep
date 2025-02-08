# Painting Package


### Testing
```
cd painting_pkg/test
python3 stroke_test.py
```

```
cd src
ros2 launch painting_pkg painting_launch.py
```

Make sure to also start the serial comm node

### Stroke Requests

Line request:
    "line, 0, 0, -500, -500"

    Means:
        Stroke Type: "line"
        Start Coordinate: (0, 0)
        End Coordinate: (-500, -500)

    The system will generate evenly spaced points along that line.

Arc request:
    "arc, -500, -500, 200, 0, 90"

    Means:
        Stroke Type: "arc"
        Center: (-500, -500)
        Radius: 200
        Start Angle: 0°
        End Angle: 90°

    The system will generate points along an arc from 0° to 90°.