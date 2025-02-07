# TODO: Test this works without user running
arduino-cli compile --fqbn arduino:avr:uno /home/luke/Projects/robots/ros-meal-prep/controller/main/main.ino
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:uno /home/luke/Projects/robots/ros-meal-prep/controller/main/main.ino
