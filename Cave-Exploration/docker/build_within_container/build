#!/usr/bin/env bash
trap : SIGTERM SIGINT

CONTAINER_NAME=cave_exp
IMAGE_NAME=ros:cave_exploration
NLOPT_VERSION="v2.7.1"

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
CAVE_EXP_DIR=$(abspath "../..")

echo $CAVE_EXP_WS_DIR
DOCKER_CATKIN_WS=$CAVE_EXP_WS_DIR
DOCKER_CAVE_EXP_DIR=$CAVE_EXP_DIR

# if [ "$#" -ne 1 ]; then
#   echo "Usage: $0 LAUNCH_FILE" >&2
#   exit 1
# fi

sudo docker run \
    -it \
    --privileged=true \
    -v $CAVE_EXP_DIR:$DOCKER_CAVE_EXP_DIR \
    --name=$CONTAINER_NAME \
    $IMAGE_NAME \
    /bin/bash \
    -c \
    "cd $DOCKER_CAVE_EXP_DIR/3rdParty/nlopt; \
    git checkout tags/$NLOPT_VERSION; \
    mkdir build && cd build; \
    cmake ..; \
    make -j; \
    sudo make install; \
    cd ..; \
    rm -rf build; \
    cd $DOCKER_CATKIN_WS; \
    catkin_make"

./back_to_host.sh
