#!/usr/bin/env bash
trap : SIGTERM SIGINT

CONTAINER_NAME=cave_exp

function abspath() {
    # generate absolute path from relative path
    # $1     : relative filename
    # return : absolute path
    if [ -d "$1" ]; then
        # dir
        (cd "$1"; pwd)
    elif [ -f "$1" ]; then
        # file
        if [[ $1 = /* ]]; then
            echo "$1"
        elif [[ $1 == */* ]]; then
            echo "$(cd "${1%/*}"; pwd)/${1##*/}"
        else
            echo "$(pwd)/$1"
        fi
    fi
}

# CAVE_EXP_DIR=$(abspath "..")
CAVE_EXP_WS_DIR=$(abspath "../../../../")
DOCKER_CATKIN_WS=$CAVE_EXP_WS_DIR
echo $DOCKER_CATKIN_WS

sudo docker start $CONTAINER_NAME
sudo docker exec -it $CONTAINER_NAME /bin/bash \
  -c "cd $DOCKER_CATKIN_WS; \
    source /opt/ros/noetic/setup.bash; \
    catkin_make" 

./back_to_host.sh

