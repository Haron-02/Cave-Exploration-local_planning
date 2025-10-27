#!/usr/bin/env bash

# 调用该文件需要传递的参数：
RUN_RVIZ=${1:-"false"} # 是否容器内启动rviz，默认false
MASTER_IP=${2:-""}     # 要多个容器的ROS之间通信，或者不使用host模式的情况下容器ROS要和宿主机ROS通信，需要指定主机IP，默认""

NLOPT_VERSION="v2.7.1"

# sudo ln -sf /bin/bash /bin/sh
# ls -l /bin/sh
# dpkg-reconfigure dash

function abspath()
{
    if [ -d "$1" ]; then
        # 如果输入的是一个目录，则进入该目录并返回绝对路径
        (cd "$1"; pwd)
    elif [ -f "$1" ]; then
        # 如果输入的是一个文件，则判断文件名是相对路径还是绝对路径，并返回绝对路径
        if [[ $1 = /* ]]; then
            # 文件名已经是绝对路径，直接返回
            echo "$1"
        elif [[ $1 == */* ]]; then
            # 文件名包含路径分隔符，说明是相对路径，需要拼接当前工作目录
            echo "$(cd "${1%/*}"; pwd)/${1##*/}"
        else
            # 文件名不包含路径分隔符，说明是当前工作目录下的文件，直接拼接即可
            echo "$(pwd)/$1"
        fi
    fi
}

# 切换到当前脚本所在目录
cd "$(dirname "$0")" || exit
# 找到工作空间目录和包目录
CONTAINER_CATKIN_WS_DIR=$(abspath "../../../../")
CONTAINER_PACKAGE_DIR=$(abspath "../../")
# echo $CONTAINER_CATKIN_WS_DIR; \
# echo $CONTAINER_PACKAGE_DIR; \

if [ -n "$MASTER_IP" ]; then
    export ROS_MASTER_URI=http://$MASTER_IP:11311; \
    export ROS_HOSTNAME=$(hostname -I | awk '{print $1}'); \
    echo ${ROS_MASTER_URI};
    echo ${ROS_HOSTNAME};
fi

cd $CONTAINER_PACKAGE_DIR/3rdParty/nlopt; \
git checkout tags/$NLOPT_VERSION; \
mkdir build && cd build; \
cmake ..; \
make -j; \
make install; \
cd ..; \
rm -rf build; \
cd $CONTAINER_CATKIN_WS_DIR; \
catkin_make; \
source devel/setup.bash; \
roslaunch exploration_manager exploration_bamboo_sim.launch is_launch_rviz:=$RUN_RVIZ
# roslaunch exploration_manager exploration_bamboo_real.launch is_launch_rviz:=$RUN_RVIZ
