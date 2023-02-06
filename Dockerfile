# This is an auto generated Dockerfile for ros:desktop-full
# generated from docker_images/create_ros_image.Dockerfile.em
FROM osrf/ros:noetic-desktop-focal

# install ros packages. Full desktop environment is not required but is easier to install
RUN apt-get update && apt-get upgrade -y
RUN  apt-get update && apt-get install -y apt-utils
RUN apt-get install -y ros-noetic-perception
#RUN rm -rf /var/lib/apt/lists/*
RUN apt-get install -y ros-noetic-cv-bridge ros-noetic-tf ros-noetic-message-filters ros-noetic-image-transport ros-noetic-image-transport*
RUN apt-get install -y libcgal-dev pcl-tools
RUN apt install -y dbus-x11 git

# install livox sdk api
RUN git clone https://github.com/Livox-SDK/Livox-SDK.git /Livox-SDK
WORKDIR /Livox-SDK/build
RUN cmake .. && make && make install

# install r3live
RUN mkdir -p /catkin_ws/src/r3live
COPY . /catkin_ws/src/r3live
RUN git clone https://github.com/Livox-SDK/livox_ros_driver.git /catkin_ws/src/livox_ros_driver

#install realsense-viewer
# update is needed for realsenese installation to work on some computers
RUN apt-get update
RUN apt-get install -y software-properties-common
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
RUN add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
RUN apt-get install -y librealsense2-dkms
RUN apt-get install -y librealsense2-utils
RUN apt-get install -y librealsense2-dev
RUN apt-get install -y librealsense2-dbg
RUN apt-get install -y ros-noetic-ddynamic-reconfigure
RUN git clone https://github.com/IntelRealSense/realsense-ros.git /catkin_ws/src/realsense_ros

WORKDIR /catkin_ws/src/realsense_ros
RUN git checkout `git tag | sort -V | grep -P "^2.\d+\.\d+" | tail -1`

WORKDIR /catkin_ws

# upgrade is needed for catkin_make to work on some computers
RUN apt-get upgrade -y

# the command below may not work while building. Will work if activated manually while inside the container.
# RUN . /opt/ros/noetic/setup.sh && catkin_make

# install vscode
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys EB3E94ADBE1229CF
RUN add-apt-repository -y "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main"
RUN apt -y install code


RUN echo "source /catkin_ws/devel/setup.bash" >> /root/bashrc
WORKDIR /
