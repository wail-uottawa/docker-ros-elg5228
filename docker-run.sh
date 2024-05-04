#!/bin/sh

docker run -it --rm --privileged --name docker-ros-elg5228 \
    -p 6080:80 \
    -e VNC_RESOLUTION=1920x1080 \
    --volume ~/OneDrive-uOttawa/Docker-ELG5228-ROS1/course_dir:/home/ubuntu/catkin_ws/src/course_dir \
    realjsk/docker-ros-noetic-vnc:20240504 \
    bash 

# Browse http://127.0.0.1:6080/
