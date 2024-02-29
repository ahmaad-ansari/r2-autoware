# Autoware Setup Guide

This guide provides step-by-step instructions on setting up an environment to run Autoware using Docker. Follow these steps carefully to ensure a smooth installation.

## Table of Contents
- [Prerequisites](#prerequisites)
- [Docker Installation for Development](#docker-installation-for-development)
  - [1. Clone Autoware Repository](#1-clone-autoware-repository)
  - [2. Install Dependencies Using Ansible (Optional)](#2-install-dependencies-using-ansible-optional)
- [Setting Up Workspace](#setting-up-workspace)
  - [1. Create Autoware Map Directory](#1-create-autoware-map-directory)
  - [2. Pull Docker Image](#2-pull-docker-image)
  - [3. Launch Docker Container](#3-launch-docker-container)
  - [5. Move to Workspace in the Container](#5-move-to-workspace-in-the-container)
  - [6. Create src Directory and Clone Repositories](#6-create-src-directory-and-clone-repositories)
  - [7. Update Dependent ROS Packages](#7-update-dependent-ros-packages)
  - [8. Build the Workspace](#8-build-the-workspace)
- [Autoware Planning Simulation](#autoware-planning-simulation)
  - [Download and Unpack Sample Map](#download-and-unpack-sample-map)
  - [Manual Downloading of Artifacts](#manual-downloading-of-artifacts)
  - [Basic Simulations](#basic-simulations)
    - [Lane Driving Scenario](#lane-driving-scenario)

Now, your environment is set up to run Autoware with a ROSMASTER R2. Ensure you follow the specific instructions for your system architecture and GPU setup.

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

## Autoware Planning Simulation

### Download and Unpack Sample Map

Download and unpack a sample map using the following commands:

```bash
gdown -O ~/autoware_map/ 'https://docs.google.com/uc?export=download&id=1499_nsbUbIeturZaDj7jhUownh5fvXHd'
unzip -d ~/autoware_map ~/autoware_map/sample-map-planning.zip
```

**Note:** Sample map is copyrighted by TIER IV, Inc.

Check if you have the `~/autoware_data` folder and required files:

```bash
cd ~/autoware_data
ls -C -w 30
```

#### Manual Downloading of Artifacts

If you do not have the required folders and files in `~/autoware_data`, you can manually download them by following the steps below:

##### 1. yabloc_pose_initializer

```bash
mkdir -p ~/autoware_data/yabloc_pose_initializer/
wget -P ~/autoware_data/yabloc_pose_initializer/ \
       https://s3.ap-northeast-2.wasabisys.com/pinto-model-zoo/136_road-segmentation-adas-0001/resources.tar.gz
```

##### 2. image_projection_based_fusion

```bash
mkdir -p ~/autoware_data/image_projection_based_fusion/
wget -P ~/autoware_data/image_projection_based_fusion/ \
       https://awf.ml.dev.web.auto/perception/models/pointpainting/v4/pts_voxel_encoder_pointpainting.onnx \
       https://awf.ml.dev.web.auto/perception/models/pointpainting/v4/pts_backbone_neck_head_pointpainting.onnx
```

##### 3. lidar_apollo_instance_segmentation

```bash
mkdir -p ~/autoware_data/lidar_apollo_instance_segmentation/
wget -P ~/autoware_data/lidar_apollo_instance_segmentation/ \
       https://awf.ml.dev.web.auto/perception/models/lidar_apollo_instance_segmentation/vlp-16.onnx \
       https://awf.ml.dev.web.auto/perception/models/lidar_apollo_instance_segmentation/hdl-64.onnx \
       https://awf.ml.dev.web.auto/perception/models/lidar_apollo_instance_segmentation/vls-128.onnx
```

##### 4. lidar_centerpoint

```bash
mkdir -p ~/autoware_data/lidar_centerpoint/
wget -P ~/autoware_data/lidar_centerpoint/ \
       https://awf.ml.dev.web.auto/perception/models/centerpoint/v2/pts_voxel_encoder_centerpoint.onnx \
       https://awf.ml.dev.web.auto/perception/models/centerpoint/v2/pts_backbone_neck_head_centerpoint.onnx \
       https://awf.ml.dev.web.auto/perception/models/centerpoint/v2/pts_voxel_encoder_centerpoint_tiny.onnx \
       https://awf.ml.dev.web.auto/perception/models/centerpoint/v2/pts_backbone_neck_head_centerpoint_tiny.onnx
```

##### 5. tensorrt_yolo

```bash
mkdir -p ~/autoware_data/tensorrt_yolo/
wget -P ~/autoware_data/tensorrt_yolo/ \
       https://awf.ml.dev.web.auto/perception/models/yolov3.onnx \
       https://awf.ml.dev.web.auto/perception/models/yolov4.onnx \
       https://awf.ml.dev.web.auto/perception/models/yolov4-tiny.onnx \
       https://awf.ml.dev.web.auto/perception/models/yolov5s.onnx \
       https://awf.ml.dev.web.auto/perception/models/yolov5m.onnx \
       https://awf.ml.dev.web.auto/perception/models/yolov5l.onnx \
       https://awf.ml.dev.web.auto/perception/models/yolov5x.onnx \
       https://awf.ml.dev.web.auto/perception/models/coco.names
```

##### 6. tensorrt_yolox

```bash
mkdir -p ~/autoware_data/tensorrt_yolox/
wget -P ~/autoware_data/tensorrt_yolox/ \
       https://awf.ml.dev.web.auto/perception/models/yolox-tiny.onnx \
       https://awf.ml.dev.web.auto/perception/models/yolox-sPlus-opt.onnx \
       https://awf.ml.dev.web.auto/perception/models/yolox-sPlus-opt.EntropyV2-calibration.table \
       https://awf.ml.dev.web.auto/perception/models/object_detection_yolox_s/v1/yolox-sPlus-T4-960x960-pseudo-finetune.onnx \
       https://awf.ml.dev.web.auto/perception/models/object_detection_yolox_s/v1/yolox-sPlus-T4-960x960-pseudo-finetune.EntropyV2-calibration.table \
       https://awf.ml.dev.web.auto/perception/models/label.txt
```

##### 7. traffic_light_classifier

```bash
mkdir -p ~/autoware_data/traffic_light_classifier/
wget -P ~/autoware_data/traffic_light_classifier/ \
       https://awf.ml.dev.web.auto/perception/models/traffic_light_classifier/v2/traffic_light_classifier_mobilenetv2_batch_1.onnx \
       https://awf.ml.dev.web.auto/perception/models/traffic_light_classifier/v2/traffic_light_classifier_mobilenetv2_batch_4.onnx \
       https://awf.ml.dev.web.auto/perception/models/traffic_light_classifier/v2/traffic_light_classifier_mobilenetv2_batch_6.onnx \
       https://awf.ml.dev.web.auto/perception/models/traffic_light_classifier/v2/traffic_light_classifier_efficientNet_b1_batch_1.onnx \
       https://awf.ml.dev.web.auto/perception/models/traffic_light_classifier/v2/traffic_light_classifier_efficientNet_b1_batch_4.onnx \
       https://awf.ml.dev.web.auto/perception/models/traffic_light_classifier/v2/traffic_light_classifier_efficientNet_b1_batch_6.onnx \
       https://awf.ml.dev.web.auto/perception/models/traffic_light_classifier/v2/lamp_labels.txt \
       https://awf.ml.dev.web.auto/perception/models/traffic_light_classifier/v3/ped_traffic_light_classifier_mobilenetv2_batch_1.onnx \
       https://awf.ml.dev.web.auto/perception/models/traffic_light_classifier/v3/ped_traffic_light_classifier_mobilenetv2_batch_4.onnx \
       https://awf.ml.dev.web.auto/perception/models/traffic_light_classifier/v3/ped_traffic_light_classifier_mobilenetv2_batch_6.onnx \
       https://awf.ml.dev.web.auto/perception/models/traffic_light_classifier/v3/lamp_labels_ped.txt
```

##### 8. traffic_light_fine_detector

```bash
mkdir -p ~/autoware_data/traffic_light_fine_detector/
wget -P ~/autoware_data/traffic_light_fine_detector/ \
       https://awf.ml.dev.web.auto/perception/models/tlr_yolox_s/v3/tlr_car_ped_yolox_s_batch_1.onnx \
       https://awf.ml.dev.web.auto/perception/models/tlr_yolox_s/v3/tlr_car_ped_yolox_s_batch_4.onnx \
       https://awf.ml.dev.web.auto/perception/models/tlr_yolox_s/v3/tlr_car_ped_yolox_s_batch_6.onnx \
       https://awf.ml.dev.web.auto/perception/models/tlr_yolox_s/v3/tlr_labels.txt
```

##### 9. traffic_light_ssd_fine_detector

```bash
mkdir -p ~/autoware_data/traffic_light_ssd_fine_detector/
wget -P ~/autoware_data

/traffic_light_ssd_fine_detector/ \
       https://awf.ml.dev.web.auto/perception/models/mb2-ssd-lite-tlr.onnx \
       https://awf.ml.dev.web.auto/perception/models/voc_labels_tl.txt
```

##### 10. tvm_utility

```bash
mkdir -p ~/autoware_data/tvm_utility/models/yolo_v2_tiny
wget -P ~/autoware_data/tvm_utility/ \
       https://autoware-modelzoo.s3.us-east-2.amazonaws.com/models/3.0.0-20221221/yolo_v2_tiny-x86_64-llvm-3.0.0-20221221.tar.gz
```

##### 11. lidar_centerpoint_tvm

```bash
mkdir -p ~/autoware_data/lidar_centerpoint_tvm/models/centerpoint_encoder
mkdir -p ~/autoware_data/lidar_centerpoint_tvm/models/centerpoint_backbone
wget -P ~/autoware_data/lidar_centerpoint_tvm/ \
       https://autoware-modelzoo.s3.us-east-2.amazonaws.com/models/3.0.0-20221221/centerpoint_encoder-x86_64-llvm-3.0.0-20221221.tar.gz \
       https://autoware-modelzoo.s3.us-east-2.amazonaws.com/models/3.0.0-20221221/centerpoint_backbone-x86_64-llvm-3.0.0-20221221.tar.gz
```

##### 12. lidar_apollo_segmentation_tvm

```bash
mkdir -p ~/autoware_data/lidar_apollo_segmentation_tvm/models/baidu_cnn
wget -P ~/autoware_data/lidar_apollo_segmentation_tvm/ \
      https://autoware-modelzoo.s3.us-east-2.amazonaws.com/models/3.0.0-20221221/baidu_cnn-x86_64-llvm-3.0.0-20221221.tar.gz
```

##### Extracting Files

```bash
tar -xf ~/autoware_data/yabloc_pose_initializer/resources.tar.gz \
       -C ~/autoware_data/yabloc_pose_initializer/
```

Now, you have manually downloaded and extracted the required artifacts for Autoware. Ensure these files are present in the specified folders for the proper functioning of the Autoware system.


### Basic Simulations

#### Lane Driving Scenario
1. **Launch Docker**
```bash
rocker -e LIBGL_ALWAYS_SOFTWARE=1 --x11 --user --volume $HOME/autoware --volume $HOME/autoware_map --volume $HOME/autoware_data -- ghcr.io/autowarefoundation/autoware-universe:latest-cuda
```
```bash
rocker --nvidia --x11 --user --volume $HOME/autoware.r2 --volume $HOME/autoware_map --volume $HOME/autoware_data -- ghcr.io/autowarefoundation/autoware-universe:latest-cuda
```

2. **Launch Autoware**

```bash
source ~/autoware/install/setup.bash
ros2 launch autoware_launch planning_simulator.launch.xml map_path:=$HOME/autoware_map/sample-map-planning vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
```

**Warning:** Do not use ~ instead of $HOME in the `map_path` argument. If ~ is used, the map will fail to load.
