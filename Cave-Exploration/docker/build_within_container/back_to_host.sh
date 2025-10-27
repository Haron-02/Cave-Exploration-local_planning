#!/usr/bin/env bash
trap : SIGTERM SIGINT

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

CAVE_EXP_WS_DIR=$(abspath "../../../../")
DOCKER_CATKIN_WS=$CAVE_EXP_WS_DIR
# echo $CAVE_EXP_WS_DIR

if [ -d $CAVE_EXP_WS_DIR/devel ]; then
    sudo rm -rf $CAVE_EXP_WS_DIR/devel
fi
sudo docker cp $(docker ps -alq):$DOCKER_CATKIN_WS/devel $CAVE_EXP_WS_DIR/

USER_NAME=$(id -u -n)
GROUP_NAME=$(id -g -n)
sudo chown -R $USER_NAME:$GROUP_NAME $CAVE_EXP_WS_DIR/
