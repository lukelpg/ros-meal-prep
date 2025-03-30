# ros-meal-prep
This is a repo for a ROS2 enviornment used for interfacing with sensors and actuators to make a meal prep robot

## Install

From top level of repo:
``` 
sudo bash installer.sh
``` 

## Run

From top level of repo:

TODO: confirm if 'sudo usermod -aG dialout $USER' was the fix for ros2 run not letting the nodes access gpio or if it was sudo colcon build

``` 
sudo colcon build
```
``` 
source install/setup.bash
```

Run all 4 nodes:
``` 
ros2 run image_processing_pkg image_processing_node
ros2 run painting_pkg painting_node
TEST NODE?? <!-- ros2 run serial_comm_pkg waypoint_publisher --> TEST NODE??
ros2 run serial_comm_pkg serial_comm
``` 

## Git Fix
``` 
git pull origin main
``` 
``` 
git reset --hard origin/main
``` 


## Robot Workspace

X 0 -> -1000 (have a bit more space)
Y 0 -> -1000 (have a bit more space)
Z 0 -> 6000


## Dev

``` 
cd src
ros2 pkg create --build-type ament_python --license Apache-2.0 <package_name>
``` 

## TODO
- Add launch files
- Make everything start on bootup
- Make path following more efficent (no decel from point to point)
- Add state machine to microcontroller so it can receive more instructions without being re-flashed
- Write algo for generating sequence of strokes from a picture (big one lol)
    - Idea: Picture -> *Something?* -> Layers? -> Shapes? -> Strokes -> Curves -> Coords. Seq. -> Inst. -> Axial Movement
    - Expand stroke types (Bezier, complex splines)

- Define inputs and outputs of ROS nodes clearly for scalability (DOING BELOW)
- 3 ROS nodes
    - ros2 run image_processing_pkg image_processing_node
        - Publishing scaled strokes one by one (to topic: 'paint_command')
    - ros2 run painting_pkg painting_node
        - Subscribed to topic 'paint_command'
        - Publishing stroke a list of waypoints one by one (to topic: 'waypoints_topic')
    - TEST NODE?? ros2 run serial_comm_pkg waypoint_publisher TEST NODE??
    - ros2 run serial_comm_pkg serial_comm
        - Subscribed to topic 'waypoints_topic'
        - Sending (over serial) MOVE instructions (waypoints) one by one and then 'GO' to signal the the complete stroke has been sent
            - Will do this for each stroke 

## Components
### Image Processing
- Input
    - Image
- Output
    - Strokes

### Painting Package
- Input
    
- Output
    

### Serial Comms
- Input
    
- Output
    

### Controller
- Input
    
- Output
    