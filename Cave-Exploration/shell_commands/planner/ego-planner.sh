#!/bin/bash
trap : SIGTERM SIGINT

SCRIPT_PATH="$(readlink -f "$0")"
WORKSPACE_DIR="/home/dreamcatcher/Traj_Planner_Compare/ego-planner_ws"
echo "Workspace directory is: ${WORKSPACE_DIR}"

gnome-terminal -t "ego_planner" --tab -- bash -c "source ${WORKSPACE_DIR}/devel/setup.bash && \
                                    roslaunch ego_planner simple_run.launch; exec bash;"

sleep 1s

gnome-terminal -t "ego_planner_rviz" --tab -- bash -c "source ${WORKSPACE_DIR}/devel/setup.bash && \
                                    roslaunch ego_planner rviz.launch; exec bash;"








