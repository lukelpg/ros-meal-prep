#!/bin/bash

# ROS 2 Humble Installer for Raspberry Pi 4 (Ubuntu 22.04)

# Make sure the system is up to date
echo "Updating system..."
sudo apt update
sudo apt upgrade -y

# Install prerequisites
echo "Installing prerequisites..."
sudo apt install -y python3
sudo apt install -y curl
sudo apt install -y gnupg2
sudo apt install -y lsb-release
sudo apt install -y build-essential
sudo apt install -y locales
sudo apt install -y bash-completion
sudo apt install -y git
sudo apt install -y wget
sudo apt install -y unzip
sudo apt install -y python3-pip
python3 -m pip install python3-colcon-common-extensions
# sudo apt install -y python3-vcstool
# sudo apt install -y python3-argparse


# Set up the locale to ensure it works properly
echo "Setting up locale..."
sudo update-locale LANG=C.UTF-8

# Add ROS 2 repository keys
echo "Adding ROS 2 repository keys..."
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo tee /etc/apt/trusted.gpg.d/ros.asc

# Add ROS 2 repository to sources list
echo "Adding ROS 2 repository..."
sudo sh -c 'echo "deb [arch=arm64] https://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list'

# Update package index
echo "Updating package index..."
sudo apt update

# Install ROS 2 Humble Desktop for ARM
echo "Installing ROS 2 Humble..."
sudo apt install -y ros-humble-desktop
sudo apt install ros-dev-tools

# Setup environment variables
echo "Setting up ROS 2 environment variables..."
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Initialize rosdep
echo "Initializing rosdep..."
sudo rosdep init
rosdep update

# Install additional development tools
echo "Installing ROS development tools..."
sudo apt install -y \
  ros-humble-ros-base \
  ros-humble-robot-state-publisher \
  ros-humble-rviz2 \
  ros-humble-teleop-twist-keyboard \
  ros-humble-navigation2

# Install dependencies using rosdep
echo "Installing ROS 2 dependencies with rosdep..."
rosdep install --from-paths src --ignore-src -r -y

# Verify the installation
echo "Verifying ROS 2 installation..."
ros2 --version

echo "ROS 2 Humble installation completed successfully!"
