#!/bin/bash
trap : SIGTERM SIGINT

SCRIPT_PATH="$(readlink -f "$0")"
WORKSPACE_DIR="$(dirname "$(dirname "$(dirname "$(dirname "$SCRIPT_PATH")")")")"
echo "Script directory is: ${WORKSPACE_DIR}"

gnome-terminal --tab -- bash -c "roscore ; exec bash"

sleep 1s

gnome-terminal -t "cave_exp" --tab -- bash -c "source ${WORKSPACE_DIR}/devel/setup.bash && \
                                  roslaunch exploration_manager exploration_virtual_cave.launch ; exec bash"

sleep 1s

gnome-terminal -t "keyboard_control" --tab -- bash -c "source ${WORKSPACE_DIR}/devel/setup.bash && \
                                    rosrun keyboard_control keyboard_control.py;exec bash;"

sleep 1s

source ./planner/modified_ego-planner.sh
