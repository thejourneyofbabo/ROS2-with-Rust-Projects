#!/bin/bash
terminator -e  " source ~/.bashrc && ros2 launch simulator simulation.launch.xml " &
sleep 7
terminator -e  " source ~/.bashrc && source install/setup.bash && ros2 launch speed_limiter speed_limiter.launch.xml "