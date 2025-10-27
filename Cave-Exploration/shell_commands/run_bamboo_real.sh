#!/bin/bash

trap : SIGTERM SIGINT

SCRIPT_PATH="$(readlink -f "$0")"
WORKSPACE_DIR="$(dirname "$(dirname "$(dirname "$(dirname "$(dirname "$SCRIPT_PATH")")")")")"

gnome-terminal --tab -- bash -c "roscore ; exec bash"
sleep 1s

#fundamental works
gnome-terminal --tab -- bash -c "source ${WORKSPACE_DIR}/DJI_ws/devel/setup.bash && \
                                roslaunch djiros djiros.launch ; exec bash"
sleep 3s

gnome-terminal --tab -- bash -c "source ${WORKSPACE_DIR}/DJI_ws/devel/setup.bash && \
                                roslaunch n3ctrl ctrl_md.launch ; exec bash"
sleep 3s
 
#fast_lio
gnome-terminal -t "fast_lio_mid360" --tab -- bash -c "source ${WORKSPACE_DIR}/Fast_Lio_Mid360_ws/devel/setup.bash && \
                                  roslaunch mapping mapping.launch rviz:=false ; exec bash"
sleep 3s

gnome-terminal -t "bamboo_exp" --tab -- bash -c "source ${WORKSPACE_DIR}/Cave_Exp_ws/devel/setup.bash && \
                                  roslaunch exploration_manager exploration_bamboo_real.launch ; exec bash"
