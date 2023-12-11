ARG ROS_DISTRO=humble

FROM husarnet/ros:$ROS_DISTRO-ros-core

SHELL ["/bin/bash", "-c"]

RUN apt update && apt upgrade -y && apt install -y \
        ros-$ROS_DISTRO-image-tools \
        ros-$ROS_DISTRO-image-transport \
        ros-$ROS_DISTRO-image-transport-plugins \
        ros-$ROS_DISTRO-teleop-twist-keyboard  && \
    apt-get autoremove -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN echo $(dpkg -s ros-$ROS_DISTRO-image-tools | grep 'Version' | sed -r 's/Version:\s([0-9]+.[0-9]+.[0-9]*).*/\1/g') >> /version.txt