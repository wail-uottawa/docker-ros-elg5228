# This Dockerfile is used to build an ROS + VNC + Tensorflow image based on Ubuntu 18.04
FROM nvidia/cuda:10.0-cudnn7-devel-ubuntu18.04 AS stage-basis

LABEL maintainer "Wail Gueaieb"
MAINTAINER Wail Gueaieb "https://github.com/wail-uottawa/docker-ros-elg5228"
ENV REFRESHED_AT 2021-02-28

# Install sudo
RUN apt-get update && \
    apt-get install -y sudo \
    apt-utils \
    xterm \
    curl \
    wget

# Configure user
ARG user=ros
ARG passwd=ros
ARG uid=1000
ARG gid=1000
ENV USER=$user
ENV PASSWD=$passwd
ENV UID=$uid
ENV GID=$gid
RUN groupadd $USER && \
    useradd --create-home --no-log-init -g $USER $USER && \
    usermod -aG sudo $USER && \
    echo "$PASSWD:$PASSWD" | chpasswd && \
    chsh -s /bin/bash $USER && \
    # Replace 1000 with your user/group id
    usermod  --uid $UID $USER && \
    groupmod --gid $GID $USER



### VNC Installation
# ref: https://github.com/ConSol/docker-headless-vnc-container
# https://blog.ouseful.info/2019/02/06/viewing-dockerised-desktops-via-an-x11-bridge-novnc/
FROM stage-basis AS stage-vnc
LABEL io.k8s.description="VNC Container with ROS with Xfce window manager" \
      io.k8s.display-name="VNC Container with ROS based on Ubuntu" \
      io.openshift.expose-services="6901:http,5901:xvnc,6006:tensorboard" \
      io.openshift.tags="vnc, ros, gazebo, tensorflow, ubuntu, xfce" \
      io.openshift.non-scalable=true

## Connection ports for controlling the UI:
# VNC port:5901
# noVNC webport, connect via http://IP:6901/?password=vncpassword
ENV DISPLAY=:1 \
    VNC_PORT=5901 \
    NO_VNC_PORT=6901
EXPOSE $VNC_PORT $NO_VNC_PORT

## Envrionment config
ENV VNCPASSWD=vncpassword
ENV HOME=/home/$USER \
    TERM=xterm \
    STARTUPDIR=/dockerstartup \
    INST_SCRIPTS=/home/$USER/install \
    NO_VNC_HOME=/home/$USER/noVNC \
    DEBIAN_FRONTEND=noninteractive \
    VNC_COL_DEPTH=24 \
    VNC_RESOLUTION=1920x1080 \
    VNC_PW=$VNCPASSWD \
    VNC_VIEW_ONLY=false
WORKDIR $HOME

## Add all install scripts for further steps
ADD ./src/common/install/ $INST_SCRIPTS/
ADD ./src/ubuntu/install/ $INST_SCRIPTS/
RUN find $INST_SCRIPTS -name '*.sh' -exec chmod a+x {} +

## Install some common tools
RUN $INST_SCRIPTS/tools.sh
ENV LANG='en_US.UTF-8' LANGUAGE='en_US:en' LC_ALL='en_US.UTF-8'

## Install xvnc-server & noVNC - HTML5 based VNC viewer
RUN $INST_SCRIPTS/tigervnc.sh
RUN $INST_SCRIPTS/no_vnc.sh

## Install firefox and chrome browser
# RUN $INST_SCRIPTS/firefox.sh
# RUN $INST_SCRIPTS/chrome.sh

## Install xfce UI
RUN $INST_SCRIPTS/xfce_ui.sh
ADD ./src/common/xfce/ $HOME/

## configure startup
RUN $INST_SCRIPTS/libnss_wrapper.sh
ADD ./src/common/scripts $STARTUPDIR
RUN $INST_SCRIPTS/set_user_permission.sh $STARTUPDIR $HOME



### ROS and Gazebo Installation
FROM stage-vnc AS stage-ros
# Install other utilities
RUN sudo apt-get update && \
    sudo apt-get install -y vim \
    tmux \
    git

# Install ROS
RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    sudo apt-get update && \
    sudo apt-get install -y ros-melodic-desktop-full && \
    sudo apt-get install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential && \
    # apt-get install -y python-rosinstall && \
    sudo rosdep init

# Setup ROS
USER $USER
RUN rosdep fix-permissions && rosdep update
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
RUN /bin/bash -c "source ~/.bashrc"



# Install Gazebo
FROM stage-ros AS stage-gazebo
USER root
RUN sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
    wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - && \
    sudo apt-get update && \
    sudo apt-get install -y gazebo9 libgazebo9-dev && \
    sudo apt-get install -y ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control

# Fixing Gazebo error
RUN sudo apt-get upgrade -y libignition-math2
# Removing unnecessary packages
# RUN sudo apt-get autoremove -y



# Install catkin_tools
# https://catkin-tools.readthedocs.io
# https://github.com/catkin/catkin_tools
FROM stage-gazebo AS stage-catkin-tools
USER root
RUN sudo apt-get install -y python3-pip
RUN /bin/bash -c 'git clone https://github.com/catkin/catkin_tools.git; cd catkin_tools; pip3 install -r requirements.txt --upgrade; sudo python3 setup.py install --record install_manifest.txt; rm -fr ../catkin_tools'



### Setting up catkin_ws
FROM stage-catkin-tools AS stage-catkin_ws
ENV CATKIN_WS=$HOME/catkin_ws
USER $USER
WORKDIR $HOME

RUN mkdir -p $CATKIN_WS/src
RUN /bin/bash -c 'source /opt/ros/melodic/setup.bash; cd $CATKIN_WS; catkin init; catkin build'
RUN /bin/bash -c 'source $CATKIN_WS/devel/setup.bash'
RUN echo "source $CATKIN_WS/devel/setup.bash" >> ~/.bashrc



### Installing ROS Control and ROS Controllers (important stacks)
FROM stage-catkin_ws AS stage-controls
USER root
RUN sudo apt-get install -y  ros-melodic-ros-control ros-melodic-ros-controllers
USER $USER
WORKDIR $HOME
RUN /bin/bash -c 'source /opt/ros/melodic/setup.bash'



### Installing PR2 Simulator
# http://wiki.ros.org/pr2_simulator
# http://wiki.ros.org/pr2_simulator/Tutorials
FROM stage-controls AS stage-pr2
USER root
RUN sudo apt-get install -y  \
    ros-melodic-pr2-simulator \
    ros-melodic-pr2-moveit-plugins \
    ros-melodic-pr2-arm-kinematics \
    ros-melodic-pr2-moveit-config \
    ros-melodic-hls-lfcd-lds-driver
# last 4 packages above are to prevent warnings in UR installation
USER $USER
WORKDIR $HOME
RUN /bin/bash -c 'source /opt/ros/melodic/setup.bash'



### Installing Husky
# http://wiki.ros.org/husky_gazebo/Tutorials/Simulating%20Husky
FROM stage-pr2 AS stage-husky
USER root
RUN sudo apt-get install -y ros-melodic-husky-simulator

USER $USER
WORKDIR $HOME

RUN /bin/bash -c 'source /opt/ros/melodic/setup.bash'
RUN export HUSKY_GAZEBO_DESCRIPTION=$(rospack find husky_gazebo)/urdf/description.gazebo.xacro
RUN echo "export HUSKY_GAZEBO_DESCRIPTION=\$(rospack find husky_gazebo)/urdf/description.gazebo.xacro" >> ~/.bashrc



### Installing Turtlebot3
# https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/
FROM stage-husky AS stage-turtlebot3
USER root

# Install Dependent ROS 1 Packages
RUN sudo apt-get update && \
    sudo apt-get upgrade -y && \
    sudo apt-get install -y \
    ros-melodic-joy ros-melodic-teleop-twist-joy \
    ros-melodic-teleop-twist-keyboard ros-melodic-laser-proc \
    ros-melodic-rgbd-launch ros-melodic-depthimage-to-laserscan \
    ros-melodic-rosserial-arduino ros-melodic-rosserial-python \
    ros-melodic-rosserial-server ros-melodic-rosserial-client \
    ros-melodic-rosserial-msgs ros-melodic-amcl ros-melodic-map-server \
    ros-melodic-move-base ros-melodic-urdf ros-melodic-xacro \
    ros-melodic-compressed-image-transport ros-melodic-rqt-image-view \
    ros-melodic-gmapping ros-melodic-navigation ros-melodic-interactive-markers

# Install TurtleBot3 Packages
USER $USER
WORKDIR $HOME

RUN /bin/bash -c 'source /opt/ros/melodic/setup.bash'
RUN /bin/bash -c 'cd $CATKIN_WS/src; git clone -b melodic-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git; git clone -b melodic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git; git clone -b melodic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git'
RUN /bin/bash -c 'cd $CATKIN_WS; source /opt/ros/melodic/setup.bash; catkin build'
RUN /bin/bash -c 'source $CATKIN_WS/devel/setup.bash'
RUN echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc

ENV HOMEBIN=$HOME/bin
RUN echo 'PATH=~/bin:$PATH' >> ~/.bashrc
ADD ./src/ros/scripts/ $HOMEBIN/
USER root
RUN find $HOMEBIN -name '*.sh' -exec chmod a+x {} +


### Tensorflow Installation may go here
# See Dockerfile-tf-jupyter for the non-working version



### Installing Kinova ROS packages
# https://github.com/Kinovarobotics/kinova-ros
FROM stage-turtlebot3 AS stage-kinova
# Install Dependent ROS 1 Packages
USER root
RUN sudo apt-get update && \
    sudo apt-get upgrade -y && \
    sudo apt-get install -y \
    ros-melodic-moveit ros-melodic-trac-ik

# Install Kinova Packages
USER $USER
WORKDIR $HOME

RUN /bin/bash -c 'source /opt/ros/melodic/setup.bash'
RUN /bin/bash -c 'cd $CATKIN_WS/src; git clone https://github.com/Kinovarobotics/kinova-ros.git kinova-ros'
RUN /bin/bash -c 'cd $CATKIN_WS; source /opt/ros/melodic/setup.bash; catkin build'
RUN /bin/bash -c 'source $CATKIN_WS/devel/setup.bash'

# To access the arm via usb
USER root
RUN sudo cp $CATKIN_WS/src/kinova-ros/kinova_driver/udev/10-kinova-arm.rules /etc/udev/rules.d/




### Installing Universal Robot Manipulators
# https://github.com/ros-industrial/universal_robot
FROM stage-kinova AS stage-ur

USER $USER
WORKDIR $HOME

RUN /bin/bash -c 'source /opt/ros/melodic/setup.bash'
RUN /bin/bash -c 'cd $CATKIN_WS/src; git clone -b melodic-devel https://github.com/ros-industrial/universal_robot.git'
RUN /bin/bash -c 'cd $CATKIN_WS; source /opt/ros/melodic/setup.bash; rosdep update; rosdep install --rosdistro melodic --ignore-src --from-paths src -r -y; catkin build'
RUN /bin/bash -c 'source $CATKIN_WS/devel/setup.bash'



### Installing Husarion Rosbot 2.0
# https://github.com/husarion/rosbot_description
# https://husarion.com/tutorials/ros-tutorials/1-ros-introduction/
# https://github.com/husarion/tutorial_pkg
FROM stage-ur AS stage-husarion

USER $USER
WORKDIR $HOME

RUN /bin/bash -c 'source /opt/ros/melodic/setup.bash'
RUN /bin/bash -c 'cd $CATKIN_WS/src; git clone https://github.com/husarion/rosbot_description.git'
RUN /bin/bash -c 'cd $CATKIN_WS; source /opt/ros/melodic/setup.bash; rosdep update; rosdep install --rosdistro melodic --ignore-src --from-paths src -r -y; catkin build'
RUN /bin/bash -c 'source $CATKIN_WS/devel/setup.bash'



### Installing Neobotix Robots
# https://github.com/neobotix/neo_simulation
# https://docs.neobotix.de/display/ROSSim/ROS-Simulation
# https://docs.neobotix.de
FROM stage-husarion AS stage-neobotix

USER $USER
WORKDIR $HOME

RUN /bin/bash -c 'source /opt/ros/melodic/setup.bash'
RUN /bin/bash -c 'cd $CATKIN_WS/src; git clone -b melodic https://github.com/neobotix/neo_simulation.git'
RUN /bin/bash -c 'cd $CATKIN_WS; source /opt/ros/melodic/setup.bash; catkin build'
RUN /bin/bash -c 'source $CATKIN_WS/devel/setup.bash'
RUN echo "export GAZEBO_MODEL_PATH=$CATKIN_WS/src/neo_simulation/models:\$GAZEBO_MODEL_PATH" >> ~/.bashrc
RUN echo "export LC_NUMERIC=\"en_US.UTF-8\" " >> ~/.bashrc

## Installing dependencies
# Dependencies for the kinematics node for MPO-700 and MPO-500
RUN /bin/bash -c 'source /opt/ros/melodic/setup.bash'
RUN /bin/bash -c 'cd $CATKIN_WS/src; git clone https://github.com/neobotix/neo_msgs.git; git clone https://github.com/neobotix/neo_srvs.git; git clone https://github.com/neobotix/neo_common.git'
RUN /bin/bash -c 'cd $CATKIN_WS; source /opt/ros/melodic/setup.bash; catkin build'
RUN /bin/bash -c 'source $CATKIN_WS/devel/setup.bash'
# Kinematics node for MPO-700 and MMO-700
RUN /bin/bash -c 'source /opt/ros/melodic/setup.bash'
RUN /bin/bash -c 'cd $CATKIN_WS/src; git clone https://github.com/neobotix/neo_kinematics_omnidrive.git'
RUN /bin/bash -c 'cd $CATKIN_WS; source /opt/ros/melodic/setup.bash; catkin build'
RUN /bin/bash -c 'source $CATKIN_WS/devel/setup.bash'
# Kinematics node for MPO-500 and MMO-500
RUN /bin/bash -c 'source /opt/ros/melodic/setup.bash'
RUN /bin/bash -c 'cd $CATKIN_WS/src; git clone https://github.com/neobotix/neo_kinematics_mecanum.git'
RUN /bin/bash -c 'cd $CATKIN_WS; source /opt/ros/melodic/setup.bash; catkin build'
RUN /bin/bash -c 'source $CATKIN_WS/devel/setup.bash'
# Neo-localization
RUN /bin/bash -c 'source /opt/ros/melodic/setup.bash'
RUN /bin/bash -c 'cd $CATKIN_WS/src; git clone https://github.com/neobotix/neo_localization.git'
RUN /bin/bash -c 'cd $CATKIN_WS; source /opt/ros/melodic/setup.bash; catkin build'
RUN /bin/bash -c 'source $CATKIN_WS/devel/setup.bash'

## Installing necessary ROS packages
USER root
RUN sudo apt-get install -y \
    ros-melodic-ros-controllers \
    ros-melodic-gazebo-ros-control \
    ros-melodic-navigation \
    ros-melodic-neo-local-planner \
    ros-melodic-eband-local-planner \
    ros-melodic-teb-local-planner \
    ros-melodic-dwa-local-planner \
    ros-melodic-openslam-gmapping \
    ros-melodic-amcl \
    ros-melodic-joy \
    ros-melodic-teleop-twist-keyboard \
    ros-melodic-tf2-sensor-msgs



### Installing RotorS
# http://wiki.ros.org/rotors_simulator
# https://github.com/ethz-asl/rotors_simulator
# https://github.com/ethz-asl/rotors_simulator/wiki
# Matlab interface: https://github.com/ethz-asl/rotors_simulator/wiki/Interfacing-RotorS-through-Matlab
FROM stage-neobotix AS stage-rotors

## Installing necessary ROS packages
USER root
RUN sudo apt-get install -y \
    ros-melodic-desktop-full \
    ros-melodic-joy \
    ros-melodic-octomap-ros \
    ros-melodic-mavlink \
    python-wstool \
    python-catkin-tools \
    protobuf-compiler \
    libgoogle-glog-dev \
    ros-melodic-control-toolbox \
    ros-melodic-mavros

## Installing RotorS
USER $USER
WORKDIR $HOME

RUN /bin/bash -c 'source /opt/ros/melodic/setup.bash'
RUN /bin/bash -c 'cd $CATKIN_WS/src; git clone https://github.com/ethz-asl/rotors_simulator.git; git clone https://github.com/ethz-asl/mav_comm.git'
RUN /bin/bash -c 'cd $CATKIN_WS; source /opt/ros/melodic/setup.bash; catkin build'
RUN /bin/bash -c 'source $CATKIN_WS/devel/setup.bash'



### Installing Panda
# https://github.com/frankaemika/franka_ros
# https://frankaemika.github.io
# https://frankaemika.github.io/docs
FROM stage-rotors AS stage-panda
USER root

RUN sudo apt-get install -y ros-melodic-libfranka ros-melodic-franka-ros
RUN /bin/bash -c 'source $CATKIN_WS/devel/setup.bash'



### Installing stdr simulator
# http://wiki.ros.org/stdr_simulator
FROM stage-panda AS stage-stdr
USER root
RUN sudo apt-get install -y libqt4-dev

USER $USER
WORKDIR $HOME

RUN /bin/bash -c 'source /opt/ros/melodic/setup.bash'
RUN /bin/bash -c 'cd $CATKIN_WS/src; git clone https://github.com/stdr-simulator-ros-pkg/stdr_simulator.git'
RUN /bin/bash -c 'cd $CATKIN_WS; source /opt/ros/melodic/setup.bash; rosdep update; rosdep install --rosdistro melodic --ignore-src --from-paths src -y; catkin build -DQT_QMAKE_EXECUTABLE=/usr/bin/qmake-qt4'
RUN /bin/bash -c 'source $CATKIN_WS/devel/setup.bash'



### Install some useful utilities
FROM stage-stdr AS stage-utilities
USER root
# Installing VScode
RUN sudo apt-get update && \
    sudo apt-get install -y software-properties-common apt-transport-https

RUN wget -q https://packages.microsoft.com/keys/microsoft.asc -O- | sudo apt-key add - && \
    sudo add-apt-repository "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main"
RUN sudo apt-get update && \
    sudo apt-get install -y code

# Installing Brave browser (https://brave.com/linux/)
RUN sudo apt-get install -y apt-transport-https curl \
    && sudo curl -fsSLo /usr/share/keyrings/brave-browser-archive-keyring.gpg https://brave-browser-apt-release.s3.brave.com/brave-browser-archive-keyring.gpg \
    && echo "deb [signed-by=/usr/share/keyrings/brave-browser-archive-keyring.gpg arch=amd64] https://brave-browser-apt-release.s3.brave.com/ stable main"|sudo tee /etc/apt/sources.list.d/brave-browser-release.list \
    && sudo apt-get update  \
    && sudo apt-get install -y brave-browser \
    && sudo apt-get install -y firefox emacs filezilla terminator 

# Installing a few extra utilities
# RUN sudo apt-get install -y firefox emacs filezilla terminator 

# Install ROS extension for VSCode
USER $USER
WORKDIR $HOME
RUN code --install-extension ms-iot.vscode-ros

# Default editor for rosed (~/.bashrc)
RUN echo "export EDITOR='emacs' " >> ~/.bashrc
# Avoid warning XDG_RUNTIME_DIR not set
RUN echo "export XDG_RUNTIME_DIR=$HOME " >> ~/.bashrc
RUN /bin/bash -c "source ~/.bashrc"



### Cleaning up and finalization
FROM stage-utilities AS stage-finalization

USER root
RUN sudo apt-get update && \
    sudo apt-get upgrade -y && \
    sudo apt-get autoremove -y
## Fixing the error "/dockerstartup/vnc_startup.sh" not found  (commands copied and pasted from above)
# configure startup
# RUN $INST_SCRIPTS/libnss_wrapper.sh
#ADD ./src/common/scripts $STARTUPDIR
RUN $INST_SCRIPTS/set_user_permission.sh $STARTUPDIR $HOME


## Switch back to $USER
USER $USER

ENTRYPOINT ["/dockerstartup/vnc_startup.sh"]
CMD ["--wait"]
