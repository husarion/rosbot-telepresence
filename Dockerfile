FROM husarnet/ros:humble-ros-base

RUN apt update && apt install -y \
        ros-humble-cv-bridge

RUN mkdir src && \
    git clone --depth 1 https://github.com/ros-misc-utilities/ffmpeg_image_transport.git src/ffmpeg_image_transport && \
    vcs import src < src/ffmpeg_image_transport/ffmpeg_image_transport.repos && \
    rosdep update --rosdistro $ROS_DISTRO && \
    rosdep install --from-paths src --ignore-src -y && \
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build

COPY ./ros2_ws/src/teleop_ui_ros /ros2_ws/src/teleop_ui_ros

RUN apt update && apt install -y \
        ros-humble-image-transport \ 
        ros-humble-image-transport-plugins


RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    source /ros2_ws/install/setup.bash && \
    rosdep update --rosdistro $ROS_DISTRO && \
    rosdep install --from-paths src --ignore-src -y && \
    colcon build

COPY ./teleop-ui.launch.py /