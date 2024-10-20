#!/usr/bin/env zsh

# Source the ROS 2 setup file
source /opt/ros/humble/setup.zsh

# Source the local setup file
source ~/ros2_turtle_project/install/setup.zsh

# Run the XML launch file
ros2 launch turtle_controller turtle_project.launch.xml
