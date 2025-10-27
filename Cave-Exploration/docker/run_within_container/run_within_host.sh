#!/usr/bin/env bash
trap : SIGTERM SIGINT

MASTER_IP=localhost

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

export ROS_MASTER_URI=http://$MASTER_IP:11311; 
export ROS_HOSTNAME=$MASTER_IP; 

roscore &
ROSCORE_PID=$!
sleep 1

source $CONTAINER_CATKIN_WS_DIR/devel/setup.bash
rviz -d $CONTAINER_PACKAGE_DIR/fuel_planner/plan_manage/config/traj.rviz &
RVIZ_PID=$!

wait $ROSCORE_PID
wait $RVIZ_PID

if [[ $? -gt 128 ]]
then
    kill $ROSCORE_PID
    kill $RVIZ_PID
fi
