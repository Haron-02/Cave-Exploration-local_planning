#!/bin/bash
trap : SIGTERM SIGINT

SCRIPT_PATH="$(readlink -f "$0")"
WORKSPACE_DIR="/home/dreamcatcher/Traj_Planner_Compare/modified_ego-planner_ws"
echo "Workspace directory is: ${WORKSPACE_DIR}"

gnome-terminal -t "modified_ego_planner" --tab -- bash -c "source ${WORKSPACE_DIR}/devel/setup.bash && \
                                    roslaunch ego_planner run_in_marsim_1.2m.launch; exec bash;"

sleep 1s

gnome-terminal -t "modified_ego_planner_rviz" --tab -- bash -c "source ${WORKSPACE_DIR}/devel/setup.bash && \
                                    roslaunch ego_planner rviz.launch; exec bash;"








