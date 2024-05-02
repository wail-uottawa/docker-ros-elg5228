#!/bin/bash

# Independent squashing tool
# https://github.com/goldmann/docker-squash

docker build -t realjsk/docker-ros-noetic-vnc:20240502 --no-cache   .

# docker build --squash -t realjsk/docker-ros-noetic-vnc:20240428 --no-cache   .

# docker build --squash --target stage-finalization -t realjsk/docker-ros-elg5228:20210908  .
# docker build --squash --target stage-finalization -t realjsk/docker-ros-elg5228:20210908 --no-cache  .
# docker build --squash -t realjsk/docker-ros-elg5228:20210908  .
