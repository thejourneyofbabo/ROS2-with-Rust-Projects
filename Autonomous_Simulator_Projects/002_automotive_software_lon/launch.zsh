#!/usr/bin/env zsh

# 현재 창을 좌우로 분할
tmux split-window -h

# 왼쪽 창(첫 번째 pane)에서 setup 실행 후 simulator 실행
tmux select-pane -t 0
tmux send-keys "source ~/.zshrc && source install/setup.zsh && ros2 launch simulator simulation.launch.xml" C-m

# 오른쪽 창(두 번째 pane)에서 setup 실행 후 7초 대기, 그 다음 speed_limiter 실행
tmux select-pane -t 1
tmux send-keys "source ~/.zshrc && source install/setup.zsh && sleep 7 && ros2 launch speed_limiter speed_limiter.launch.xml" C-m

# 왼쪽 창으로 포커스 이동
tmux select-pane -t 0
