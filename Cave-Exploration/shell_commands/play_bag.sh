#!/bin/bash

SCRIPT_PATH="$(readlink -f "$0")"
WORKSPACE_DIR="$(dirname "$(dirname "$(dirname "$(dirname "$SCRIPT_PATH")")")")"

BAG_ADDR="${WORKSPACE_DIR}/src/Cave-Exploration/rosbag"
# 获取当前时间并格式化为文件名
BAG_NAME="20240724_213159.bag"

echo "Script directory is: ${WORKSPACE_DIR}"
echo "File name is: ${BAG_NAME}"

gnome-terminal -t "roscore" -- bash -c \
"roscore; 
exec bash"

sleep 1s

gnome-terminal -t "bag_record" -- bash -c \
"source ${WORKSPACE_DIR}/devel/setup.bash &&
roslaunch exploration_manager rviz_sim.launch; 
exec bash"

gnome-terminal -t "bag_record" -- bash -c \
"rosbag play ${BAG_ADDR}/${BAG_NAME}; 
exec bash"
