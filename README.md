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

Example running stepper node:
``` 
ros2 run stepper_pkg stepper
ros2 run stepper_pkg faceTrack
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
ros2 pkg create --build-type ament_python --license Apache-2.0 <package_name>
``` 
