***Docker image with ROS, Gazebo, Xfce4 VNC Desktop, and several robot packages***

**Maintainer:** *Wail Gueaieb*

![GitHub all releases](https://img.shields.io/github/downloads/wail-uottawa/docker-ros-elg5228/total)
![GitHub last commit (branch)](https://img.shields.io/github/last-commit/wail-uottawa/docker-ros-elg5228/master)
[![GitHub license](https://img.shields.io/github/license/wail-uottawa/docker-ros-elg5228)](https://github.com/wail-uottawa/docker-ros-elg5228/blob/master/LICENSE)

<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table of Contents**

- [Overview](#overview)
- [Quick Startup](#quick-startup)
    - [Running the Image](#running-the-image)
    - [Connecting to the Image](#connecting-to-the-image)
        - [Connecting Through Web Browser](#connecting-through-web-browser)
        - [Connecting Through VNC Viewer](#connecting-through-vnc-viewer)
    - [Stopping the Image](#stopping-the-image)
- [Getting the Docker Image](#getting-the-docker-image)
    - [Pulling the Docker Image from Docker Hub](#pulling-the-docker-image-from-docker-hub)
    - [Building the Docker Image Locally](#building-the-docker-image-locally)
- [Running the Container](#running-the-container)
    - [Connect & Control](#connect--control)
    - [Environment Settings](#environment-settings)
        - [Using root (user id `0`)](#using-root-user-id-0)
        - [Using user and group id of host system](#using-user-and-group-id-of-host-system)
        - [Overriding VNC and container environment variables](#overriding-vnc-and-container-environment-variables)
            - [Example: Overriding the VNC password](#example-overriding-the-vnc-password)
            - [Example: Overriding the VNC resolution](#example-overriding-the-vnc-resolution)
        - [Mounting a local directory to the container](#mounting-a-local-directory-to-the-container)
- [ROS Catkin Workspace](#ros-catkin-workspace)
- [Installed Robots](#installed-robots)
    - [Fixed Manipulators](#fixed-manipulators)
    - [Wheeled Mobile Robots](#wheeled-mobile-robots)
    - [Wheeled Mobile Manipulators](#wheeled-mobile-manipulators)
    - [Aerial robots](#aerial-robots)
- [Utilities](#utilities)
- [Bin](#bin)
- [Acknowledgment](#acknowledgment)
- [Disclaimer](#disclaimer)

<!-- markdown-toc end -->


# Overview

This is a docker image to support teaching ROS-based robotic courses (including ELG 5228: Mobile Robots at the University of Ottawa) and to help researchers and hobbyists to experiment with a number of robots. It provides enough power and flexibility to cover various robotic topics (e.g., navigation, control, path planning, manipulators, wheeled mobile robots, aerial robots, etc.) with the ease to add more as needed. It comes with the following main components:
* ROS Melodic installed on Ubuntu 18.04
* Gazebo 9
* Xfce4 VNC Desktop to facilitate remote access
* ROS packages of a number of robots, as detailed blow

The Dockerfile is inspired by that of henry2423/docker-ros-vnc: [https://github.com/henry2423/docker-ros-vnc](https://github.com/henry2423/docker-ros-vnc). Most of the documentation for that repository is still valid here, except:
  * Only ROS Melodic is supported (ROS Kinetic and Lunar are not).
  * Tensorflow and Jupyter are not installed.
  * ROS-melodic-desktop-full is used instead of the original version.
  
# Quick Startup
## Running the Image
Probably, the easiest way to run the Docker image is to run the provided shell script file `docker-run.sh` at a command line while Docker (or Docker Desktop) is running. 

---
<span style="color:red">**WARNING:**</span> All changes made to any file/directory within the file system of a docker container are not permanent. They are lost once the container is stopped. To avoid this problem, the `docker-run.sh` script maps a local folder `~/MyGDrive/Docker-ELG5228/course_dir` on your computer onto another folder `/home/ros/catkin_ws/src/course_dir` in the docker container. It is highly recommended that you dedicate a local folder on your computer as a ROS working folder throughout the course. It can have any name and path (for example: `c:/Courses/Mobile-robotics/ROS-Work` for Windows hosts or `/Users/john/ELG5228/ROS-Work` for Mac hosts). To be even safer, you might want to have this folder as part of a cloud drive that is automatically synchronized on your local machine (such as Google Drive, OneDrive, etc.) Inside the file `docker-run.sh`, replace `~/MyGDrive/Docker-ELG5228/course_dir` with the path to your dedicated local folder. That way, each time you run the dockerimage through `docker-run.sh` your local dedicated folder is automatically mapped onto `/home/ros/catkin_ws/src/course_dir` in the docker container. As such, whenever you make changes on your local dedicated folder or/and on `/home/ros/catkin_ws/src/course_dir` from within the container, those changes remain permanent on the local drive and are automatically made visible from within the container at `/home/ros/catkin_ws/src/course_dir` every time the container is run. 

---

---
<span style="color:red">**NOTE:**</span> The first time you run the script in `docker-run.sh`, it will take a relatively long time to pull the image from Docker hub, due to its large size. However, subsequent runs should be much faster since the image will be cached locally by Docker.

---

## Connecting to the Image
Successfully running `docker-run.sh` takes you to a shell command inside the image. However, this doesn't allow you to run graphical applications. To do so, you need to connect to the image from within your host machine either through a web browser or a VNC viewer, such as tigerVNC Viewer ([https://tigervnc.org](https://tigervnc.org)).

### Connecting Through Web Browser
Simply point your web browser to either of the following two URLs;
* connect via __noVNC HTML5 full client__: [`http://localhost:6901/vnc.html`](http://localhost:6901/vnc.html), default password: `vncpassword` 
* connect via __noVNC HTML5 lite client__: [`http://localhost:6901/?password=vncpassword`](http://localhost:6901/?password=vncpassword) 

### Connecting Through VNC Viewer
Point your VNC viewer as follows:
* connect via __VNC viewer `localhost:5901`__, default password: `vncpassword`
Once connected, it might be best to run your VNC viewer in full screen mode, for a better experience. 

---
<span style="color:red">**NOTE:**</span> If you connect to the image via a vnc viewer, it is important to remember to terminate the vnc connection before stopping the docker image, otherwise you may experience some difficulty (vnc lock) next time you try to connect to the image via a vnc viewer. Sometimes, when the vnc connection is not terminated properly, it causes some vnc lock files to remain behind inside the server (the docker image in this case). These files are `/tmp/.X1.lock` and `/tmp/.X11-unix/X1`. The proper way to terminate a vnc connection is to follow *either* of the following options:
* From the host computer, close the vnc viewer application (by closing its window for example).
* From within the docker image, run the command `vncserver -kill :1`, which terminates the connection from the server. This example terminates Display 1 (:1).
* The third option (better leave it as the last resort) is to delete files `/tmp/.X1.lock` and `/tmp/.X11-unix/X1` inside the container. 

---

More details about connecting to the image can be found down in Section [Connect & Control](#connect--control).

## Stopping the Image
After finishing working with the docker image, and once you properly disconnected your vnc connection in case you connected via a VNC viewer, you can stop the image in one of the following methods:
* Graphically through Docker Desktop
* From the command line on your host computer by running `docker stop IMAGE_ID`, where `IMAGE_ID` is the ID of the image you want to stop. Another way is to run `docker stop $(docker ps -a -q)`, which will stop *all* docker images running on your computer.

# Getting the Docker Image
The docker image can be either pulled directly from Docker Hub, or built locally on your personal computer. The former method may be much more convenient. 

## Pulling the Docker Image from Docker Hub
Currently, the docker image lives in a Docker Hub repository [realjsk/docker-ros-elg5228](https://hub.docker.com/r/realjsk/docker-ros-elg5228 "realjsk/docker-ros-elg5228"). It can be pulled using the docker command:

	docker pull realjsk/docker-ros-elg5228:20210323

## Building the Docker Image Locally
The source files to build the docker image on a local computer are stored at the Github repository [https://github.com/wail-uottawa/docker-ros-elg5228](https://github.com/wail-uottawa/docker-ros-elg5228).

1. Start by cloning the repository:  
   `git clone https://github.com/wail-uottawa/docker-ros-elg5228.git`
2. Then, cd to the directory including the file `Dockerfile` and (with the docker server running) build the image:  
   `docker build -t realjsk/docker-ros-elg5228:20210323  .` (note the dot at the end)  
   If you want, you may change `realjsk/docker-ros-elg5228` and `20210323` to any other image name and tag of your choice, respectively.  
   This can also be done by running the provided shell script `docker-build.sh` using the command:  
   `sh docker-build.sh`  
   Feel free to edit the shell script if you want.

# Running the Container
The container is developed under xfce-docker-container source, which makes it accessible through xfce-vnc or no-vnc (via http vnc service). In the following, it is assumed that the `name:tag` of the docker image is the same as the default one mentioned above (`realjsk/docker-ros-elg5228:20210323`). If you used a different one, please make the necessary changes to the proceeding commands.

- Run command with a mapping to local port `5901` (vnc protocol) and `6901` (vnc web access):

      `docker run -d -p 5901:5901 -p 6901:6901 realjsk/docker-ros-elg5228`

- Another alternative to connect to the container is to use the interactive mode `-it` and `bash`
      
      `docker run -it -p 5901:5901 -p 6901:6901 realjsk/docker-ros-elg5228 bash`

## Connect & Control
Once it is running, you can connect to the container in a number of ways to be able to run GUI applications, such as Gazebo and Rviz:
* connect via __VNC viewer `localhost:5901`__, default password: `vncpassword`
* connect via __noVNC HTML5 full client__: [`http://localhost:6901/vnc.html`](http://localhost:6901/vnc.html), default password: `vncpassword` 
* connect via __noVNC HTML5 lite client__: [`http://localhost:6901/?password=vncpassword`](http://localhost:6901/?password=vncpassword) 

The default username and password in the container is `ros:ros`.

The default password for the `sudo` command is `ros`.

## Environment Settings

### Using root (user id `0`)
Add the `--user` flag to your docker run command. For example:

    docker run -it --user root -p 5901:5901 realjsk/docker-ros-elg5228

### Using user and group id of host system
In Unix-like host systems, you may add the `--user` flag to your docker run command. For example:

    docker run -it -p 5901:5901 --user $(id -u):$(id -g) realjsk/docker-ros-elg5228:20210323

Note that it may not be always possible to map the user and group ids of the host system to those of the container, which is 1000:1000. In that case, you may want to try overriding the VNC and container envirenment variables, as explained below (see [Overriding VNC and container environment variables](#overriding-vnc-and-container-environment-variables)).

### Overriding VNC and container environment variables
The following VNC environment variables can be overwritten within the docker run command to customize the desktop environment inside the container:
* `VNC_COL_DEPTH`, default: `24`
* `VNC_RESOLUTION`, default: `1920x1080`
* `VNC_PW`, default: `vncpassword`
* `USER`, default: `ros`
* `PASSWD`, default: `ros`

#### Example: Overriding the VNC password
Simply overwrite the value of the environment variable `VNC_PW`. For example, in
the docker run command:

    docker run -it -p 5901:5901 -p 6901:6901 -e VNC_PW=vncpassword realjsk/blimp:20210323 

#### Example: Overriding the VNC resolution
Simply overwrite the value of the environment variable `VNC_RESOLUTION`. For example, in the docker run command:

    docker run -it -p 5901:5901 -p 6901:6901 -e VNC_RESOLUTION=800x600 realjsk/blimp:20210323

### Mounting a local directory to the container
Docker enables the mapping between directories on the host system and the container through the `--volume` directive. For example, the following command maps the host user/group with the container, and also maps a few directories between the host and the container. Note that in such cases, the host serves as the master while the container is the slave. With the following command, for instance, the user account in the container will be the same as the host account.

      docker run -it -p 5901:5901 \
        --user $(id -u):$(id -g) \
        --volume /etc/passwd:/etc/passwd \
        --volume /etc/group:/etc/group \
        --volume /etc/shadow:/etc/shadow \
        --volume /home/ros/Desktop:/home/ros/Desktop:rw \
        realjsk/blimp:20210323

You can learn more about volumes on this designated [docker reference page](https://docs.docker.com/storage/volumes/).

# ROS Catkin Workspace 
The container comes with a catkin workspace already set up. By default, the path for the catkin workspace is  
`/home/ros/catkin_ws`

Some ROS packages are installed in the catkin workspace, including the [Airship Simulation](https://github.com/robot-perception-group/airship_simulation) repository, as shall be described later.

In order for users to write their own ROS packages without running the risk of interfering with the pre-installed packages in this catkin workspace, it is recommended to include all user packages in a mapped directory inside the `src` directory of the catkin workspace. For example, the user may want to dedicate a folder on the host machine for his own ROS packages. Let's say that the path to this folder is `/home/john/work_dir`, then it can be mapped to a folder inside the container's catkin workspace by adding the following part to the docker run command:  
`--volume /home/john/work_dir:/home/ros/catkin_ws/src/work_dir:rw`

Be aware that you may have to specify the path to the host folder differently in case the host is running Windows as an operating system.

# Installed Robots

## Fixed Manipulators
* Kinova's Jaco, Jaco2, and Micro arms [[Official page](https://www.kinovarobotics.com) | [Github](https://github.com/Kinovarobotics/kinova-ros)]
* Universal Robots (UR3, UR5, UR10) [[Official page](https://www.universal-robots.com) | [Github](https://github.com/ros-industrial/universal_robot)]
* Franka's Panda [[Official page](https://www.franka.de) | [Github](https://frankaemika.github.io) | [Franka ROS](https://frankaemika.github.io/docs/franka_ros.html)]
* PR2 [[ROS Wiki](http://wiki.ros.org/pr2_simulator) | [PR2 Simulator Tutorial](http://wiki.ros.org/pr2_simulator/Tutorials)]

## Wheeled Mobile Robots
* Turtlebot3 [[Official page](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)]
* Husky [[Official page](https://clearpathrobotics.com/husky-unmanned-ground-vehicle-robot/) | [Github](https://github.com/husky)]
* Husarion Rosbot 2.0 [[Official page](https://husarion.com) | [Github](https://github.com/husarion)]
* Neobotix robots [[Official page](https://docs.neobotix.de) | [Github](https://github.com/neobotix)]
	* Neobotix differential drive robots (MP-400 and MP-500)
	* Neobotix omnidirectional robot with Mecanum wheels (MPO-500)
	* Neobotix omnidirectional robot with Omni-Drive-Modules (MPO-700)

## Wheeled Mobile Manipulators
* Neobotix mobile platforms [[Official page](https://docs.neobotix.de) | [Github](https://github.com/neobotix)]
	* MM-400: Neobotix mobile platform MP-400 with a robot arm from PILZ, Schunk or Panda 
	* MMO-500: Neobotix mobile platform MPO-500 with a robot arm from Universal Robots, Kuka, Rethink Robotics or Schunk
	* MMO-700: Neobotix mobile platform MPO-700 with a robot arm from Universal Robots, Kuka, Rethink Robotics or Schunk

## Aerial robots
* RotorS: A MAV gazebo simulator. It provides some multirotor models such as the AscTec Hummingbird, the AscTec Pelican, or the AscTec Firefly, and more.
[[ROS Wiki](http://wiki.ros.org/rotors_simulator) | [Github](https://github.com/ethz-asl/rotors_simulator) | [Github Wiki](https://github.com/ethz-asl/rotors_simulator/wiki)]

# Utilities
Editors: vi (and vim), Emacs, VS Code
Web browsers: Firefox, Brave
FTP clients: Filezilla

# Bin
killall_gazebo

# Acknowledgment
Credit goes primarily to the maintainers of the following projects:

* [henry2423/docker-ros-vnc](https://github.com/henry2423/docker-ros-vnc) - developed the base Dockerfile used for this image
* [ConSol/docker-headless-vnc-container](https://github.com/ConSol/docker-headless-vnc-container) - developed the ConSol/docker-headless-vnc-container

# Disclaimer
The main purpose of this repository and docker image is to facilitate instructors and researchers efforts in experimenting and conducting realistic simulations of various types of robotic systems. However, they come with no warranty. Please use them at your own discretion. 

I am no docker expert. It is very likely that the generated docker image and the provided `Dockerfile` are by no means optimal. 
