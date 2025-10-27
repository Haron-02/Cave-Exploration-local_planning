#!/bin/bash
trap : SIGTERM SIGINT

SCRIPT_PATH="$(readlink -f "$0")"
WORKSPACE_DIR="$(dirname "$(dirname "$(dirname "$(dirname "$SCRIPT_PATH")")")")"
echo "Script directory is: ${WORKSPACE_DIR}"

gnome-terminal --tab -- bash -c "roscore ; exec bash"

sleep 1s

CURRENT_TIME=$(date +%s)
FORMATTED_TIME=$(date -d "@$CURRENT_TIME" +"%Y_%m_%d-%H_%M_%S")

SCENE="hotel"
DATA_PATH="${WORKSPACE_DIR}/src/Cave-Exploration/exploration_data_files/${SCENE}/${FORMATTED_TIME}"

mkdir -p "$DATA_PATH"

gnome-terminal -t "bag_record" --tab -- bash -c "source ${WORKSPACE_DIR}/devel/setup.bash && \
roslaunch exploration_manager rosbag_record.launch file_dir:=${DATA_PATH}; 
exec bash"

gnome-terminal -t "cave_exp" --tab -- bash -c "source ${WORKSPACE_DIR}/devel/setup.bash && \
                                  roslaunch exploration_manager exploration_hotel.launch start_time:=$CURRENT_TIME scene:=$SCENE; exec bash"

sleep 1s

gnome-terminal -t "keyboard_control" --tab -- bash -c "source ${WORKSPACE_DIR}/devel/setup.bash && \
                                    rosrun keyboard_control keyboard_control.py;exec bash;"

sleep 1s

gnome-terminal -t "robot_status" --tab -- bash -c "source ${WORKSPACE_DIR}/devel/setup.bash && \
                                    rosrun robot_status robot_status.py;exec bash;"

sleep 1s


source ./planner/modified_ego-planner_1.2m.sh

# source ./planner/fast-planner.sh

# source ./planner/ego-planner.sh

# source ./planner/ego-planner-v2.sh
