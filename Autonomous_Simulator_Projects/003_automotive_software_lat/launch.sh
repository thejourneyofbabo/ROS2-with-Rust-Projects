#!/bin/bash
terminator -e  " source ~/.bashrc && source install/setup.bash && ros2 launch simulator simulation.launch.xml "
