#!/bin/sh

docker run -it --rm --privileged --name docker-ros-elg5228 \
       -p 5901:5901 \
       -p 6901:6901 \
       -e VNC_RESOLUTION=1920x1080 \
       --volume ~/OneDrive-uOttawa/Docker-ELG5228-ROS1/course_dir:/home/ros/catkin_ws/src/course_dir \
       realjsk/docker-ros-elg5228:20210908 \
       bash

