S# Autoware and ROSMASTER R2 Setup Guide

This guide provides step-by-step instructions on setting up an environment to run Autoware along with Yahboom's ROSMASTER R2 using Docker. Follow these steps carefully to ensure a smooth installation.

## Prerequisites

1. **Git:** Ensure that Git is installed on your system.
2. **For NVIDIA Jetson devices:** Install JetPack version 5.0 or higher.

## Docker Installation for Development

### 1. Clone Autoware Repository

```bash
git clone https://github.com/autowarefoundation/autoware.git
cd autoware
```

### 2. Install Dependencies Using Ansible (Optional)

**Note:** Exercise caution with this method. Ensure you have read and confirmed all steps in the Ansible configuration before proceeding.

```bash
./setup-dev-env.sh docker
```

Logout and log back in if needed after installing dependencies.

## Setting Up Workspace

### 1. Create Autoware Map Directory

```bash
mkdir ~/autoware_map
```

### 2. Pull Docker Image

```bash
docker pull ghcr.io/autowarefoundation/autoware-universe:latest-cuda
```

### 3. Launch Docker Container

#### For non-NVIDIA GPU or arm64 architecture computers:

```bash
rocker -e LIBGL_ALWAYS_SOFTWARE=1 --x11 --user --volume $HOME/autoware --volume $HOME/autoware_map -- ghcr.io/autowarefoundation/autoware-universe:latest-cuda
```

### 5. Move to Workspace in the Container

```bash
cd autoware
```

### 6. Create src Directory and Clone Repositories

```bash
mkdir src
vcs import src < autoware.repos
```

### 7. Update Dependent ROS Packages

```bash
sudo apt update
sudo rosdep update
sudo rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```

### 8. Build the Workspace

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

If any build issues occur, refer to the Troubleshooting section in the Autoware documentation.

Now, your environment is set up to run Autoware with a ROSMASTER R2. Ensure you follow the specific instructions for your system architecture and GPU setup.
