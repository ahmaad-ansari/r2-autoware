# ROSMASTER R2 Setup Guide

This guide provides step-by-step instructions on setting up an environment to run ROSMASTER R2 using Docker. Follow these steps carefully to ensure a smooth installation.

## Table of Contents
- [Accessing ROSMASTER R2 Repository with Docker](#accessing-rosmaster-r2-repository-with-docker)
  - [Enable X Server Access](#1-enable-x-server-access)
  - [Run Docker Container](#2-run-docker-container)
  - [Change to Home Directory](#3-change-to-home-directory)
  - [Clone Autoware Repository](#4-clone-autoware-repository)
  - [Create Source Directory and Import Repositories](#5-create-source-directory-and-import-repositories)
  - [Build Necessary Packages](#6-build-necessary-packages)
  - [Source the Workspace](#7-source-the-workspace)

Now, your environment is set up to run Autoware with a ROSMASTER R2. Ensure you follow the specific instructions for your system architecture and GPU setup.

## Accessing ROSMASTER R2 Repository with Docker

### 1. Enable X Server Access

```bash
xhost +
```

This command grants permission for other users to connect to your X server. In this case, it allows the Docker container to access the graphical display.

### 2. Run Docker Container

```bash
docker run -it --network autoware_rosmaster_r2_bridge --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /home/jetson/temp:/root/yahboomcar_ros2_ws/temp \
    -v /home/jetson/rosboard:/root/rosboard \
    -v /home/jetson/maps:/root/maps \
    -v /dev/bus/usb/001/010:/dev/bus/usb/001/010 \
    -v /dev/bus/usb/001/011:/dev/bus/usb/001/011 \
    --device=/dev/astradepth \
    --device=/dev/astrauvc \
    --device=/dev/video0 \
    --device=/dev/myserial \
    --device=/dev/rplidar \
    --device=/dev/input \
    -p 9090:9090 -p 8888:8888 \
    --name rosmaster-r2-container yahboomtechnology/ros-foxy:4.0.0 /bin/bash
```

### ?. Change .bashrc to Reflect R2 Model

### 3. Change to Home Directory

```bash
cd ~
```

This command changes the working directory to the user's home directory inside the Docker container.

### 4. Clone Autoware Repository

```bash
git clone https://github.com/autowarefoundation/autoware.git
cd autoware
```

Clones the Autoware repository from GitHub and navigates into the cloned directory.

### 5. Create Source Directory and Import Repositories

```bash
mkdir src
vcs import src < autoware_msgs.repos
```

Creates a `src` directory and imports Autoware repositories using the `vcs` tool.

### 6. Build Necessary Packages

```bash
colcon build --symlink-install --packages-select autoware_auto_control_msgs --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Builds the necessary packages, specifically selecting `autoware_auto_control_msgs`, with additional CMake arguments.

### 7. Source the Workspace

```bash
source ~/autoware/install/setup.bash
```

Sources the setup file to set up the environment for using the Autoware workspace and built packages.

Now, you have set up the Docker container to access the ROSMASTER R2 Repository, cloned the Autoware repository, built necessary packages, and sourced the workspace for further usage.
