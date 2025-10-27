#!/bin/bash
trap : SIGTERM SIGINT

SCRIPT_PATH="$(readlink -f "$0")"
WORKSPACE_DIR="$(dirname "$(dirname "$(dirname "$(dirname "$SCRIPT_PATH")")")")"
echo "Script directory is: ${WORKSPACE_DIR}"


gnome-terminal -t "cave_exp" --tab -- bash -c "source ${WORKSPACE_DIR}/devel/setup.bash && \
                                  roslaunch exploration_manager exploration_box.launch ; exec bash"

gnome-terminal -t "keyboard_control" --tab -- bash -c "source ${WORKSPACE_DIR}/devel/setup.bash && \
                                    rosrun keyboard_control keyboard_control.py;exec bash;"

