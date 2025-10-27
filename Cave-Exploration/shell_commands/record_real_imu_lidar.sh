#!/bin/bash

SCRIPT_PATH="$(readlink -f "$0")"
WORKSPACE_DIR="$(dirname "$(dirname "$(dirname "$(dirname "$(dirname "$SCRIPT_PATH")")")")")"
BAG_ADDR="/home/zxx02/workspace/Cave_Exp_ws/src/Cave-Exploration/exploration_data_files/rosbag/record_real_imu_lidar.bag"

# echo "Script directory is: ${WORKSPACE_DIR}"


gnome-terminal -t "bag_record" -- bash -c \
  "source ${WORKSPACE_DIR}/Cave_Exp_ws/devel/setup.bash &&
  roslaunch exploration_manager rosbag_real_imu_lidar.launch bagfile_path:=${BAG_ADDR};
  exec bash"

