FROM ros:noetic-perception

ARG CATKIN_WS_DIR

SHELL ["/bin/bash", "-c"]
ENV CATKIN_WS=$CATKIN_WS_DIR
ENV CATKIN_PACKAGE_DIR=$CATKIN_WS_DIR/src/Cave-Exploration

RUN echo "The value of CATKIN_WS_DIR is $CATKIN_WS_DIR" && \
    sh -c '. /etc/lsb-release && echo "deb http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list' && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    cp /etc/apt/sources.list /etc/apt/sources.list.bak && \
    sed -i 's/archive.ubuntu.com/mirrors.aliyun.com/g' /etc/apt/sources.list && \
    sed -i 's/security.ubuntu.com/mirrors.aliyun.com/g' /etc/apt/sources.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    libglew-dev \
    libglfw3-dev \
    libdw-dev \
    libtbb-dev \
    libarmadillo-dev\ 
    libgoogle-glog-dev \
    libatlas-base-dev \
    libeigen3-dev \
    libsuitesparse-dev \
    libignition-common3-graphics-dev \
    libignition-common3-profiler-dev \
    ros-${ROS_DISTRO}-rviz \
    ros-${ROS_DISTRO}-navigation \
    ros-${ROS_DISTRO}-lms1xx \
    ros-${ROS_DISTRO}-message-filters \
    ros-${ROS_DISTRO}-tf* \
    git \
    pcl-tools \
    tmux \
    cmake \
    wget \
    build-essential \
    python3-tk \
    python3-matplotlib \
    python3-catkin-tools && \
    rm -rf /var/lib/apt/lists/*

# COPY . $CATKIN_PACKAGE_DIR

WORKDIR $CATKIN_WS
