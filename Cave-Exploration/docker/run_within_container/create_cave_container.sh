#!/usr/bin/env bash

# 要使用的镜像名和要创建的容器名
CONTAINER_NAME=cave_exp
IMAGE_NAME=ros:cave_exploration

# 宿主机的ip，和容器数据卷挂载目录地址
MASTER_IP=localhost
VOLUME_BIND_WITHIN_HOST="/home/dreamcatcher/docker/container_data/cave_container_data_1"
VOLUME_BIND_WITHIN_CONTAINER="/data"

# 如果容器内的工作空间直接使用和宿主机一样的绝对路径, 那就不用修改以下两个变量, 方便起见, 一般直接使用和宿主机一样的绝对路径即可
# 否则请输入实际目录, 且该实际目录要和dockerfile中指定的一致
CONTAINER_CATKIN_WS_DIR=""
CONTAINER_PACKAGE_DIR=""


# 如果linux系统默认的/bin/sh的链接目标不是/bin/bash，如果不是用的bash解释器的话下面的函数定义可能报错
# 可通过执行：
# echo $SHELL        # 输出当前使用的 Shell
# ls -l /bin/sh      # 查看 /bin/sh 的链接目标
# 查看默认的链接，如果不是链接到/bin/bash，可通过执行
# sudo ln -sf /bin/bash /bin/sh
# 调整默认链接

# 但最好就是直接使用 bash *.sh 的命令运行文件，就能使用bash作为解释器！

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
CATKIN_WS_DIR=$(abspath "../../../../")
PACKAGE_DIR=$(abspath "../../")

if [ -z "$CONTAINER_CATKIN_WS_DIR" ]; then
    CONTAINER_CATKIN_WS_DIR=$CATKIN_WS_DIR
fi

if [ -z "$CONTAINER_PACKAGE_DIR" ]; then
    CONTAINER_PACKAGE_DIR=$PACKAGE_DIR
fi

# echo $CATKIN_WS_DIR
# echo $CONTAINER_CATKIN_WS_DIR
# echo $PACKAGE_DIR
# echo $CONTAINER_PACKAGE_DIR

xhost +

# 1.如果需要多个docker容器交互，容器使用自定义的bridge模式的网络，
# 接着主机ROS环境变量设置主机本身的ip：
#     export ROS_MASTER_URI=http://<主机本身的ip>:11311; 
#     export ROS_HOSTNAME=<主机本身的ip>; 
# 然后启动rviz即可可视化。

# sudo docker run \
#     -it \
#     --rm \
#     --privileged=true \
#     -v $VOLUME_BIND_WITHIN_HOST:$VOLUME_BIND_WITHIN_CONTAINER \
#     -v $CATKIN_WS_DIR:$CONTAINER_CATKIN_WS_DIR \
#     --device=/dev/dri \
#     --group-add video \
#     --volume=/tmp/.X11-unix:/tmp/.X11-unix  \
#     --env="DISPLAY=$DISPLAY" \
#     --env="QT_X11_NO_MITSHM=1" \
#     --name=$CONTAINER_NAME \
#     $IMAGE_NAME \
#     /bin/bash -c \
#     "
#     bash $CONTAINER_PACKAGE_DIR/docker/run_within_container/run_within_container.sh true $MASTER_IP; 
#     "

# 2.只有一个docker容器的时候，容器直接使用host模式的网络即可，主机启动rviz即可可视化。
sudo docker run \
    -it \
    --rm \
    --privileged=true \
    --network=host \
    -v $VOLUME_BIND_WITHIN_HOST:$VOLUME_BIND_WITHIN_CONTAINER \
    -v $CATKIN_WS_DIR:$CONTAINER_CATKIN_WS_DIR \
    --device=/dev/dri \
    --group-add video \
    --volume=/tmp/.X11-unix:/tmp/.X11-unix  \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --name=$CONTAINER_NAME \
    $IMAGE_NAME \
    /bin/bash -c \
    "
    bash $CONTAINER_PACKAGE_DIR/docker/run_within_container/run_within_container.sh false $MASTER_IP; 
    "
    
# # 3.显示用的Nvidia显卡，如果想要容器内部直接启动rviz，则需要将容器内外的nvidia-driver版本对齐
# NVIDIA_DRIVER_VERSION="$(glxinfo | grep "OpenGL version string" | rev | cut -d" " -f1 | rev)"
# sudo docker run \
#     -it \
#     --rm \
#     --privileged=true \
#     -v $VOLUME_BIND_WITHIN_HOST:$VOLUME_BIND_WITHIN_CONTAINER \
#     -v $CATKIN_WS_DIR:$CONTAINER_CATKIN_WS_DIR \
#     --device=/dev/dri \
#     --group-add video \
#     --volume=/tmp/.X11-unix:/tmp/.X11-unix  \
#     --env="DISPLAY=$DISPLAY" \
#     --env="QT_X11_NO_MITSHM=1" \
#     --name=$CONTAINER_NAME \
#     $IMAGE_NAME \
#     /bin/bash -c \
#     "wget http://us.download.nvidia.com/XFree86/Linux-x86_64/$NVIDIA_DRIVER_VERSION/NVIDIA-Linux-x86_64-$NVIDIA_DRIVER_VERSION.run; \
#     mv NVIDIA-Linux-x86_64-$NVIDIA_DRIVER_VERSION.run NVIDIA-DRIVER.run; \
#     apt update; \
#     apt install -y kmod; \
#     sh NVIDIA-DRIVER.run -a -N --ui=none --no-kernel-module --accept-license --no-questions; \
#     rm -f NVIDIA-DRIVER.run; \
#     bash $CONTAINER_PACKAGE_DIR/docker/run_within_container/run_within_container.sh true $MASTER_IP;
#     "

