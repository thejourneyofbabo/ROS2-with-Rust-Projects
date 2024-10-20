#!/usr/bin/env bash

# Exit immediately if a command exits with a non-zero status.
set -e

# Path to the ROS2 setup file
ROS_SETUP_FILE="/opt/ros/humble/setup.bash"
# Path to the project's setup file
# PROJECT_SETUP_FILE="${HOME}/ros2_turtlesim_project/install/setup.bash"
PROJECT_SETUP_FILE="./install/setup.bash"

# Source ROS2 setup
if [ -f "$ROS_SETUP_FILE" ]; then
    source "$ROS_SETUP_FILE"
else
    echo "Error: ROS2 setup file not found at $ROS_SETUP_FILE"
    exit 1
fi

# Source project setup
if [ -f "$PROJECT_SETUP_FILE" ]; then
    source "$PROJECT_SETUP_FILE"
else
    echo "Error: Project setup file not found at $PROJECT_SETUP_FILE"
    exit 1
fi

# Run the launch file
ros2 launch turtlesim_project turtlesim_project.launch.xml
