#!/bin/bash

SCRIPT_PATH="$(readlink -f "$0")"
WORKSPACE_DIR="$(dirname "$(dirname "$(dirname "$(dirname "$SCRIPT_PATH")")")")"

BAG_ADDR="${WORKSPACE_DIR}/src/Cave-Exploration/rosbag"
# 获取当前时间并格式化为文件名
BAG_NAME=$(date +"next_point_%Y%m%d_%H%M%S")

echo "Script directory is: ${WORKSPACE_DIR}"
echo "File name is: ${BAG_NAME}"

gnome-terminal -t "bag_record" -- bash -c \
"source ${WORKSPACE_DIR}/devel/setup.bash &&
roslaunch exploration_manager rosbag_record_next_point.launch file_dir:=${BAG_ADDR} file_name:=${BAG_NAME}; 
exec bash"
