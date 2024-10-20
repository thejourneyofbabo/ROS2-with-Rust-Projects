#!/bin/bash
terminator -e  " source ~/.bashrc && ros2 launch simulator simulation.launch.xml " &
sleep 7
terminator -e  " source ~/.bashrc && ros2 launch evaluation evaluation.launch.xml " #&
# sleep 3
# terminator  -e  " ros2 launch final_simulator simulation.launch "
