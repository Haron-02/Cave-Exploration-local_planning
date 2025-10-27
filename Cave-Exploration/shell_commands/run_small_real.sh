#!/bin/bash
trap : SIGTERM SIGINT

SCRIPT_PATH="$(readlink -f "$0")"
WORKSPACE_DIR="$(dirname "$(dirname "$(dirname "$(dirname "$(dirname "$SCRIPT_PATH")")")")")"
# echo "Script directory is: ${WORKSPACE_DIR}"

gnome-terminal -t "fast_lio_mid360" --tab -- bash -c "source ${WORKSPACE_DIR}/Fast_Lio_Mid360_ws/devel/setup.bash && \
                                  roslaunch mapping mapping.launch rviz:=false ; exec bash"
sleep 3s

gnome-terminal -t "cave_exp" --tab -- bash -c "source ${WORKSPACE_DIR}/Cave_Exp_ws/devel/setup.bash && \
                                  roslaunch exploration_manager exploration_real_small.launch ; exec bash"
sleep 3s

