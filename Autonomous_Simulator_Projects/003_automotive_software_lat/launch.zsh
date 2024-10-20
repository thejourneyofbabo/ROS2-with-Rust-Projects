#!/usr/bin/env zsh

# Ensure this script is executable with: chmod +x launch.zsh

# Commands to run in the left and right panes
LEFT_CMD="echo 'Left pane: Add your command here' && zsh"
RIGHT_CMD="source ~/.zshrc && source install/setup.zsh && ros2 launch simulator simulation.launch.xml"

# Check if we're already inside a tmux session
if [ -n "$TMUX" ]; then
    # We're inside tmux. Split the current pane horizontally
    tmux split-window -h

    # Send commands to the left pane (the original pane)
    tmux send-keys -t 0 "$LEFT_CMD" C-m

    # Send commands to the right pane (the newly created pane)
    tmux send-keys -t 1 "$RIGHT_CMD" C-m

    # Optional: Select the right pane
    tmux select-pane -t 1
else
    # We're not in tmux. Print an error message
    echo "Error: This script should be run from within a tmux session."
    exit 1
fi
