FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO=humble
ENV TURTLEBOT3_MODEL=waffle

RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates curl gnupg2 lsb-release locales \
    && locale-gen en_US en_US.UTF-8 \
    && curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
       | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
       > /etc/apt/sources.list.d/ros2.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
       ros-humble-ros-base \
       ros-humble-navigation2 \
       ros-humble-nav2-bringup \
       ros-humble-slam-toolbox \
       ros-humble-ros2-control \
       ros-humble-ros2-controllers \
       ros-humble-moveit \
       ros-humble-gazebo-ros-pkgs \
       ros-humble-gazebo-ros2-control \
       ros-humble-turtlebot3 \
       ros-humble-turtlebot3-gazebo \
       ros-humble-turtlebot3-navigation2 \
       ros-humble-xacro \
       ros-humble-robot-localization \
       python3-colcon-common-extensions \
       python3-pytest \
       python3-rosdep \
       build-essential \
       git \
    && rm -rf /var/lib/apt/lists/*

COPY docker/entrypoint.sh /homeagent_entrypoint.sh
RUN chmod +x /homeagent_entrypoint.sh

WORKDIR /workspace
ENTRYPOINT ["/homeagent_entrypoint.sh"]
CMD ["bash"]
