#!/usr/bin/env zsh

# Ensure this script is executable with: chmod +x launch.zsh

# Set the path to your ROS2 workspace
ROS2_WS="~/Programming/ROS2/automotive_software_lecture/Autonomous_Simulator_Projects/001_automotive_software_simulator"

# Commands to run in each pane
CMD1="cd $ROS2_WS && source ~/.zshrc && source /opt/ros/humble/setup.zsh && source install/setup.zsh && ros2 launch simulator simulation.launch.xml"
CMD2="cd $ROS2_WS && source ~/.zshrc && source /opt/ros/humble/setup.zsh && source install/setup.zsh && ros2 launch evaluation evaluation.launch.xml"

# Check if we're already inside a tmux session
if [ -n "$TMUX" ]; then
    # We're inside tmux. Split the current pane horizontally
    tmux split-window -h

    # Send the first command to the left pane (the original pane)
    tmux send-keys -t 0 "$CMD1" C-m

    # Wait for 7 seconds
    sleep 7

    # Send the second command to the right pane (the newly created pane)
    tmux send-keys -t 1 "$CMD2" C-m

    # Optional: Select the right pane
    tmux select-pane -t 1
else
    # We're not in tmux. Print an error message
    echo "Error: This script should be run from within a tmux session."
    exit 1
fi
