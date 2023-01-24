# This is an auto generated Dockerfile for ros:desktop-full
# generated from docker_images/create_ros_image.Dockerfile.em
FROM osrf/ros:noetic-desktop-focal

# install ros packages. Full desktop environment is not required but is easier to install
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y apt-utils
RUN apt-get install -y ros-noetic-perception
#RUN rm -rf /var/lib/apt/lists/*
RUN apt-get install -y ros-noetic-cv-bridge ros-noetic-tf ros-noetic-message-filters ros-noetic-image-transport ros-noetic-image-transport*
RUN apt-get install -y libcgal-dev pcl-tools
RUN apt install -y dbus-x11 git

# install livox sdk api
RUN git clone https://github.com/Livox-SDK/Livox-SDK.git /Livox-SDK
WORKDIR /Livox-SDK/build
RUN cmake .. && make && make install

# source ROS funcs
# RUN source /opt/ros/noetic/setup.bash

# install livox ros driver
# RUN mkdir -p /livox_ws/src
# RUN git clone https://github.com/Livox-SDK/livox_ros_driver.git /livox_ws/src
# WORKDIR /livox_ws
# RUN . /opt/ros/noetic/setup.sh && catkin_make
# SHELL ["/bin/bash", "-c"]
# RUN source /livox_ws/devel/setup.bash
# SHELL ["/bin/sh", "-c"]

# install r3live
RUN mkdir -p /catkin_ws/src/r3live
COPY . /catkin_ws/src/r3live
RUN git clone https://github.com/Livox-SDK/livox_ros_driver.git /catkin_ws/src/livox_ros_driver
WORKDIR /catkin_ws
RUN . /opt/ros/noetic/setup.sh && catkin_make

WORKDIR /
# SHELL ["/bin/bash", "-c"]
# RUN source /catkin_ws/devel/setup.bash
# SHELL ["/bin/sh", "-c"]