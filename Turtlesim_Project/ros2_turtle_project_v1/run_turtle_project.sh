#!/bin/bash

# Source the ROS 2 setup file
source /opt/ros/humble/setup.bash

# Source the local setup file
source ~/ros2_turtle_project/install/setup.bash

# Run the XML launch file
ros2 launch turtle_controller turtle_project.launch.xml
