# This Dockerfile is used to build an ROS + VNC + Tensorflow image based on Ubuntu 18.04
FROM nvidia/cuda:10.0-cudnn7-devel-ubuntu18.04 AS Stage-Basis

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


### Install VScode
FROM Stage-Basis AS Stage-VScode
RUN sudo apt-get update && \
    sudo apt-get install -y software-properties-common apt-transport-https

RUN wget -q https://packages.microsoft.com/keys/microsoft.asc -O- | sudo apt-key add - && \
    sudo add-apt-repository "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main"
RUN sudo apt-get update && \
    sudo apt-get install -y code


### VNC Installation
FROM Stage-VScode AS Stage-VNC
LABEL io.k8s.description="VNC Container with ROS with Xfce window manager" \
      io.k8s.display-name="VNC Container with ROS based on Ubuntu" \
      io.openshift.expose-services="6901:http,5901:xvnc,6006:tnesorboard" \
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
RUN $INST_SCRIPTS/firefox.sh
RUN $INST_SCRIPTS/chrome.sh

## Install xfce UI
RUN $INST_SCRIPTS/xfce_ui.sh
ADD ./src/common/xfce/ $HOME/

## configure startup
RUN $INST_SCRIPTS/libnss_wrapper.sh
ADD ./src/common/scripts $STARTUPDIR
RUN $INST_SCRIPTS/set_user_permission.sh $STARTUPDIR $HOME


### ROS and Gazebo Installation
FROM Stage-VNC AS Stage-ROS
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


# Install Gazebo
FROM Stage-ROS AS Stage-Gazebo
RUN sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
    wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - && \
    sudo apt-get update && \
    sudo apt-get install -y gazebo9 libgazebo9-dev && \
    sudo apt-get install -y ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control

# Fixing Gazebo error
RUN sudo apt-get upgrade -y libignition-math2
# Removing unnecessary packages
# RUN sudo apt-get autoremove -y

# Setup ROS
USER $USER
RUN rosdep fix-permissions && rosdep update
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
RUN /bin/bash -c "source ~/.bashrc"



### Setting up catkin_container_ws
FROM Stage-Gazebo AS Stage-catkin_ws1
ENV CATKIN_WS1=$HOME/catkin_container_ws
USER $USER
WORKDIR $HOME

RUN mkdir -p $CATKIN_WS1/src
RUN /bin/bash -c 'source /opt/ros/melodic/setup.bash; cd $CATKIN_WS1; catkin_make'
RUN echo "source $CATKIN_WS1/devel/setup.bash --extend" >> ~/.bashrc



### Installing Turtlebot3
FROM Stage-catkin_ws1 AS Stage-Turtlebot3
ENV HOMEBIN=$HOME/bin 
USER $USER
WORKDIR $HOME

RUN echo 'PATH=~/bin:$PATH' >> ~/.bashrc

RUN /bin/bash -c 'cd $CATKIN_WS1/src; git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git; git clone https://github.com/ROBOTIS-GIT/turtlebot3.git; git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git'
RUN /bin/bash -c 'source /opt/ros/melodic/setup.bash; cd $CATKIN_WS1; catkin_make'
RUN /bin/bash -c 'source ~/.bashrc'

ADD ./src/ros/scripts/ $HOMEBIN/
USER root
RUN find $HOMEBIN -name '*.sh' -exec chmod a+x {} +


### Tensorflow Installation may go here
# See Dockerfile-tf-jupyter for the non-working version


FROM Stage-Turtlebot3 AS Stage-Finalization
### Cleaning up
USER root
## Fixing the error "/dockerstartup/vnc_startup.sh" not found  (commands copied and pasted from above)
# configure startup
# RUN $INST_SCRIPTS/libnss_wrapper.sh
#ADD ./src/common/scripts $STARTUPDIR
RUN $INST_SCRIPTS/set_user_permission.sh $STARTUPDIR $HOME


### Switch to root user to install additional software
USER $USER

ENTRYPOINT ["/dockerstartup/vnc_startup.sh"]
CMD ["--wait"]
