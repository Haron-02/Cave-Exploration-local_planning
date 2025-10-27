#!/bin/bash
trap : SIGTERM SIGINT

SCRIPT_PATH="$(readlink -f "$0")"
WORKSPACE_DIR="/home/dreamcatcher/Traj_Planner_Compare/ego-planner-v2_ws/src/EGO-Planner-v2/swarm-playground/main_ws"
echo "Workspace directory is: ${WORKSPACE_DIR}"

gnome-terminal -t "ego_planner" --tab -- bash -c "source ${WORKSPACE_DIR}/devel/setup.bash && \
                                    roslaunch ego_planner run_in_sim.launch; exec bash;"

sleep 1s

gnome-terminal -t "ego_planner_rviz" --tab -- bash -c "source ${WORKSPACE_DIR}/devel/setup.bash && \
                                    roslaunch ego_planner rviz.launch; exec bash;"








