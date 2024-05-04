#!/bin/sh

sudo docker run -it --rm --privileged --name docker-ros-elg5228 \
       -p 6080:80 \
       -e VNC_RESOLUTION=1920x1080 \
       --volume ~/course_dir:/home/ubuntu/catkin_ws/src/course_dir \
       realjsk/docker-ros-noetic-vnc:20240504 \
       bash

