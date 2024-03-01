# Autoware and ROSMASTER R2 Setup Guide

This guide provides step-by-step instructions on setting up an environment to run Autoware along with Yahboom's ROSMASTER R2 using Docker. Follow these steps carefully to ensure a smooth installation.

## Table of Contents
- [Connecting Containers](#connecting-containers)
  - [Autoware Container](#autoware-container)
  - [R2 Container](#r2-container)
- [Setting up Docker Network](#setting-up-docker-network)
- [Setting ROS Domain ID](#setting-ros-domain-id)

## Setting up Docker Network

Execute the following command to set up the Docker network named `autoware_r2_bridge` on the host:
```bash
docker network create autoware_r2_bridge
```

## Connecting Containers

1. **Autoware Container**
  A. ARM64
    ```bash
    rocker -e LIBGL_ALWAYS_SOFTWARE=1 --x11 --user --volume $HOME/autoware --volume $HOME/autoware_map --volume $HOME/autoware_data --network autoware_r2_bridge --name autoware-container ghcr.io/autowarefoundation/autoware-universe:latest-cuda
    ```

3. **R2 Container**
   ```bash
    xhost + && docker run -it --network autoware_rosmaster_r2_bridge --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" -v /tmp/.X11-unix:/tmp/.X11-unix -v /home/jetson/temp:/root/yahboomcar_ros2_ws/temp -v /home/jetson/rosboard:/root/rosboard -v /home/jetson/maps:/root/maps -v /dev/bus/usb/001/010:/dev/bus/usb/001/010 -v /dev/bus/usb/001/011:/dev/bus/usb/001/011 --device=/dev/astradepth --device=/dev/astrauvc --device=/dev/video0 --device=/dev/myserial --device=/dev/rplidar --device=/dev/input -p 9090:9090 -p 8888:8888 --name rosmaster-r2-container yahboomtechnology/ros-foxy:4.0.0 /bin/bash
    ```

## Setting ROS Domain ID

After connecting the containers, set the ROS_DOMAIN_ID to the same value in both containers. Execute the following commands:
```bash
export ROS_DOMAIN_ID=##
source ~/.bashrc
```

Check if both containers have the correct ROS_DOMAIN_ID:
```bash
printenv | grep ROS
```

The ROS_DOMAIN_ID
