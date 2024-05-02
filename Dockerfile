# A Docker image to provide HTML5 VNC interface to access Ubuntu LXDE + ROS, based on dorowu/ubuntu-desktop-lxde-vnc
# https://hub.docker.com/r/tiryoh/ros-desktop-vnc
# https://github.com/Tiryoh/docker-ros-desktop-vnc

# FROM tiryoh/ros-desktop-vnc:noetic-amd64-20240421T0248 AS stage-basis
FROM tiryoh/ros-desktop-vnc:noetic-amd64-20240428T0225 AS stage-basis
# FROM osrf/ros:noetic-desktop-full AS stage-basis

LABEL maintainer "Wail Gueaieb"
MAINTAINER Wail Gueaieb "https://github.com/wail-uottawa/docker-ros-elg5228"
ENV REFRESHED_AT 2024-04-28

#######################################################

RUN apt-get update && apt-get install -y \
    cmake \
    curl \
    libglu1-mesa-dev \
    nano \
    python3-pip \
    python3-pydantic \
    ros-noetic-catkin \
    ros-noetic-gazebo-ros \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-joint-state-publisher \
    ros-noetic-robot-localization \
    ros-noetic-plotjuggler-ros \
    ros-noetic-robot-state-publisher \
    ros-noetic-rqt-tf-tree \
    ros-noetic-slam-toolbox \
    ros-noetic-turtlebot3 \
    ros-noetic-turtlebot3-msgs \
    ros-noetic-twist-mux \
    ros-noetic-usb-cam \
    ros-noetic-xacro \
    ruby-dev \
    rviz \
    tmux \
    wget \
    xorg-dev \
    zsh

# To use catkin build instead of catkin_make
RUN apt-get update && apt-get install -y \
    python3-wstool \
    python3-rosinstall-generator \
    python3-catkin-lint \
    python3-pip \
    python3-catkin-tools
RUN pip3 install osrf-pycommon

#######################################################

# Install ubuntu packages
RUN apt-get update && \
    apt-get install -y sudo \
    apt-utils \
    xterm \
    curl \
    wget \
    dos2unix \
    evince \
    viewnior \
    vim tmux git \
    emacs filezilla terminator 



### Setting up catkin_ws
FROM stage-basis AS stage-catkin_ws
# ARG user=ubuntu
# ENV USER=$user
ENV HOME=/home/ubuntu
ENV CATKIN_WS=$HOME/catkin_ws

#USER $USER
WORKDIR $HOME

RUN mkdir -p $CATKIN_WS/src
RUN /bin/bash -c 'source /opt/ros/noetic/setup.bash; cd $CATKIN_WS; catkin init; catkin build'
RUN /bin/bash -c 'source $CATKIN_WS/devel/setup.bash'
RUN echo "source $CATKIN_WS/devel/setup.bash" >> ~/.bashrc



### Installing Turtlebot3
# https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation
FROM stage-catkin_ws AS stage-turtlebot3

# Install Dependent ROS Noetic Packages
RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y \
    ros-noetic-joy ros-noetic-teleop-twist-joy \
    ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
    ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
    ros-noetic-rosserial-python ros-noetic-rosserial-client \
    ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
    ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
    ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
    ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers

# Install TurtleBot3 Packages
RUN apt-get install -y \
    ros-noetic-turtlebot3-msgs  ros-noetic-turtlebot3

RUN /bin/bash -c 'source /opt/ros/noetic/setup.bash'
RUN /bin/bash -c 'cd $CATKIN_WS/src; git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git'
RUN /bin/bash -c 'cd $CATKIN_WS; source /opt/ros/noetic/setup.bash; catkin build'
RUN /bin/bash -c 'source $CATKIN_WS/devel/setup.bash'
RUN echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc



### Installing Husky
# http://wiki.ros.org/husky_gazebo/Tutorials/Simulating%20Husky
# https://wiki.ros.org/Robots/Husky
# https://github.com/husky
FROM stage-turtlebot3 AS stage-husky

RUN apt-get install -y ros-noetic-husky-simulator ros-noetic-husky-control ros-noetic-husky-desktop ros-noetic-husky-msgs ros-noetic-husky-navigation ros-noetic-husky-viz     

RUN /bin/bash -c 'source /opt/ros/noetic/setup.bash'
RUN echo "export HUSKY_GAZEBO_DESCRIPTION=\$(rospack find husky_gazebo)/urdf/description.gazebo.xacro" >> ~/.bashrc
# Enabling the SICK LMS1XX LIDAR
# More customization environment variables are found at
# http://wiki.ros.org/husky_bringup/Tutorials/Customize%20Husky%20Configuration
RUN echo "export HUSKY_LMS1XX_ENABLED=true" >> ~/.bashrc



### Installing Husarion Panther
# https://github.com/husarion/panther_ros
# https://husarion.com/manuals/
# https://husarion.com/software/docker/robots/
FROM stage-husky AS stage-husarion

RUN /bin/bash -c 'export HUSARION_ROS_BUILD_TYPE=simulation'
RUN /bin/bash -c 'source /opt/ros/noetic/setup.bash'
RUN /bin/bash -c 'cd $CATKIN_WS; git clone https://github.com/husarion/panther_ros.git src/panther_ros'
RUN /bin/bash -c 'vcs import $CATKIN_WS/src < $CATKIN_WS/src/panther_ros/panther/panther.repos'
RUN /bin/bash -c 'mv $CATKIN_WS/src/panther_ros/panther_description/ $CATKIN_WS/src/panther_description'
RUN /bin/bash -c 'mv $CATKIN_WS/src/panther_ros/panther_gazebo/ $CATKIN_WS/src/panther_gazebo'
RUN /bin/bash -c 'rm -rf $CATKIN_WS/src/panther_ros'
RUN /bin/bash -c 'cd $CATKIN_WS; source /opt/ros/noetic/setup.bash; rosdep update --rosdistro noetic; rosdep install --from-paths src -y -i; source /opt/ros/noetic/setup.bash; catkin build'
RUN /bin/bash -c 'source /opt/ros/noetic/setup.bash'
RUN /bin/bash -c 'source $CATKIN_WS/devel/setup.bash'



### Installing Neobotix Robots
# https://neobotix-docs.de/ros/ros1/simulation.html
# https://github.com/neobotix/neo_simulation
# https://docs.neobotix.de/display/ROSSim/ROS-Simulation
# https://docs.neobotix.de
FROM stage-husarion AS stage-neobotix

RUN /bin/bash -c 'source /opt/ros/noetic/setup.bash'
RUN /bin/bash -c 'cd $CATKIN_WS/src; git clone -b noetic https://github.com/neobotix/neo_simulation.git'
RUN /bin/bash -c 'cd $CATKIN_WS; source /opt/ros/noetic/setup.bash; catkin build'
RUN /bin/bash -c 'source $CATKIN_WS/devel/setup.bash'
RUN echo "export GAZEBO_MODEL_PATH=$CATKIN_WS/src/neo_simulation/models:\$GAZEBO_MODEL_PATH" >> ~/.bashrc

## Installing dependencies
# Dependencies for the kinematics node for MPO-700 and MPO-500
RUN /bin/bash -c 'source /opt/ros/noetic/setup.bash'
RUN /bin/bash -c 'cd $CATKIN_WS/src; git clone https://github.com/neobotix/neo_msgs.git; git clone https://github.com/neobotix/neo_srvs.git; git clone https://github.com/neobotix/neo_common.git'
RUN /bin/bash -c 'cd $CATKIN_WS; source /opt/ros/noetic/setup.bash; catkin build'
RUN /bin/bash -c 'source $CATKIN_WS/devel/setup.bash'
# Kinematics node for MPO-700 and MMO-700
RUN /bin/bash -c 'source /opt/ros/noetic/setup.bash'
RUN /bin/bash -c 'cd $CATKIN_WS/src; git clone https://github.com/neobotix/neo_kinematics_omnidrive.git'
RUN /bin/bash -c 'cd $CATKIN_WS; source /opt/ros/noetic/setup.bash; catkin build'
RUN /bin/bash -c 'source $CATKIN_WS/devel/setup.bash'
# Kinematics node for MPO-500 and MMO-500
RUN /bin/bash -c 'source /opt/ros/noetic/setup.bash'
RUN /bin/bash -c 'cd $CATKIN_WS/src; git clone https://github.com/neobotix/neo_kinematics_mecanum.git'
RUN /bin/bash -c 'cd $CATKIN_WS; source /opt/ros/noetic/setup.bash; catkin build'
RUN /bin/bash -c 'source $CATKIN_WS/devel/setup.bash'
# Neo-localization
RUN /bin/bash -c 'source /opt/ros/noetic/setup.bash'
RUN /bin/bash -c 'cd $CATKIN_WS/src; git clone https://github.com/neobotix/neo_localization.git'
RUN /bin/bash -c 'cd $CATKIN_WS; source /opt/ros/noetic/setup.bash; catkin build'
RUN /bin/bash -c 'source $CATKIN_WS/devel/setup.bash'

## Installing necessary ROS packages
RUN apt-get install -y \
    ros-noetic-ros-controllers \
    ros-noetic-gazebo-ros-control \
    ros-noetic-navigation \
    ros-noetic-neo-local-planner \
    ros-noetic-map-server \
    ros-noetic-move-base \
    ros-noetic-openslam-gmapping \
    ros-noetic-amcl \
    ros-noetic-joy \
    ros-noetic-teleop-twist-keyboard \
    ros-noetic-tf2-sensor-msgs



### Installing Kinova ROS packages
# https://github.com/Kinovarobotics/kinova-ros/tree/noetic-devel/kinova_gazebo
FROM stage-neobotix AS stage-kinova
# Install Dependent ROS Noetic Packages
RUN apt-get install -y \
    ros-noetic-gazebo-ros* \
    ros-noetic-gazebo-ros-control \
    ros-noetic-ros-controllers \
    ros-noetic-moveit \
    ros-noetic-trac-ik \
    ros-noetic-trac-ik-kinematics-plugin

# Install Kinova Packages
RUN /bin/bash -c 'source /opt/ros/noetic/setup.bash'
RUN /bin/bash -c 'cd $CATKIN_WS/src; git clone -b noetic-devel https://github.com/Kinovarobotics/kinova-ros.git kinova-ros'
RUN /bin/bash -c 'cd $CATKIN_WS; source /opt/ros/noetic/setup.bash; catkin build'
RUN /bin/bash -c 'source $CATKIN_WS/devel/setup.bash'

# To access the arm via usb
RUN sudo cp $CATKIN_WS/src/kinova-ros/kinova_driver/udev/10-kinova-arm.rules /etc/udev/rules.d/



### Installing Universal Robot Manipulators
# https://github.com/ros-industrial/universal_robot
FROM stage-kinova AS stage-ur

# Using apt
# RUN apt-get install -y ros-noetic-universal-robots

# Building from Source
RUN /bin/bash -c 'source /opt/ros/noetic/setup.bash'
RUN /bin/bash -c 'cd $CATKIN_WS/src; git clone -b noetic-devel https://github.com/ros-industrial/universal_robot.git'
RUN /bin/bash -c 'cd $CATKIN_WS; source /opt/ros/noetic/setup.bash; rosdep update; rosdep install --rosdistro noetic --ignore-src --from-paths src; catkin build'
RUN /bin/bash -c 'source $CATKIN_WS/devel/setup.bash'



### Installing Panda
# https://frankaemika.github.io/docs
FROM stage-ur AS stage-panda
USER root

RUN sudo apt-get install -y ros-noetic-libfranka ros-noetic-franka-ros
RUN /bin/bash -c 'source $CATKIN_WS/devel/setup.bash'
