#!/bin/bash
trap : SIGTERM SIGINT

SCRIPT_PATH="$(readlink -f "$0")"
WORKSPACE_DIR="/home/dreamcatcher/Traj_Planner_Compare/fast-planner_ws"
echo "Workspace directory is: ${WORKSPACE_DIR}"

gnome-terminal -t "fast_planner" --tab -- bash -c "source ${WORKSPACE_DIR}/devel/setup.bash && \
                                    roslaunch plan_manage kino_replan_marsim.launch; exec bash;"

sleep 1s

gnome-terminal -t "fast_planner_rviz" --tab -- bash -c "source ${WORKSPACE_DIR}/devel/setup.bash && \
                                    roslaunch plan_manage rviz.launch; exec bash;"


