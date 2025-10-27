#!/usr/bin/env bash

# 先去修改 create_cave_container.sh 和 run_within_host.sh 的必要变量!!!
# 1. 主机ip。需要用到(使用桥接网络和宿主机的ROS通信、多容器之间的ROS通信)就改为自己的主机IP，不需要用到就填"".

    # 创建一个新的tmux会话
    tmux new-session -d -s cave_exp

    # 在第一个窗格启动容器
    tmux send-keys -t cave_exp "bash ./run_within_container/create_cave_container.sh" Enter

    # 在第二个窗格启动rviz
    tmux split-window -v -t cave_exp
    tmux send-keys -t cave_exp "bash ./run_within_container/run_within_host.sh" Enter

    # 设置窗格布局为垂直分割
    tmux select-layout -t cave_exp even-vertical

    # 运行tmux会话
    tmux attach -t cave_exp

