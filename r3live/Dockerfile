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



#install realsense-viewer
RUN sudo apt-get install -y software-properties-common
RUN sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
RUN  sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
RUN  sudo apt-get install -y librealsense2-dkms
RUN  sudo apt-get install -y librealsense2-utils
RUN  sudo apt-get install -y librealsense2-dev
RUN  sudo apt-get install -y librealsense2-dbg



RUN sudo apt-get install -y ros-noetic-ddynamic-reconfigure





RUN git clone https://github.com/IntelRealSense/realsense-ros.git /catkin_ws/src/realsense_ros



WORKDIR /catkin_ws/src/realsense_ros
RUN git checkout `git tag | sort -V | grep -P "^2.\d+\.\d+" | tail -1`


WORKDIR /catkin_ws




#RUN . /opt/ros/noetic/setup.sh &&  catkin_make clean
#RUN . /opt/ros/noetic/setup.sh &&  catkin_make  #-DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
#RUN . /opt/ros/noetic/setup.sh &&  catkin_make install
RUN . /opt/ros/noetic/setup.sh && catkin_make






RUN sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys EB3E94ADBE1229CF
RUN sudo add-apt-repository -y "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main"
RUN sudo apt -y install code



RUN echo "source /catkin_ws/devel/setup.bash" >> /root/bashrc





WORKDIR /
# SHELL ["/bin/bash", "-c"]
# RUN source /catkin_ws/devel/setup.bash
# SHELL ["/bin/sh", "-c"]
