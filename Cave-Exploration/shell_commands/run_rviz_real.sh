#!/bin/bash
trap : SIGTERM SIGINT

SCRIPT_PATH="$(readlink -f "$0")"
WORKSPACE_DIR="$(dirname "$(dirname "$(dirname "$(dirname "$(dirname "$SCRIPT_PATH")")")")")"
# echo "Script directory is: ${WORKSPACE_DIR}"

gnome-terminal -t "cave_exp" --tab -- bash -c "source ${WORKSPACE_DIR}/Cave_Exp_ws/devel/setup.bash && \
                                  roslaunch exploration_manager rviz_real.launch ; exec bash"
