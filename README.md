# Autoware Documentation

## Table of Contents
1. [Launching Autoware with Docker](#launching-autoware-with-docker)
2. [Ackermann Control Command ROS2 Topic](#ackermann-control-command-ros2-topic)
3. [TF Topic - `tf2_msgs/msg/TFMessage`](#tf-topic---tf2_msgsmsgtfmessage)

## Launching Autoware with Docker

To launch Autoware using Docker, follow these steps:

1. Run the following command to start Autoware in a Docker container:

    ```bash
    rocker -e LIBGL_ALWAYS_SOFTWARE=1 --x11 --user --volume $HOME/autoware --volume $HOME/autoware_map --volume $HOME/autoware_data -- ghcr.io/autowarefoundation/autoware-universe:latest-cuda
    ```

   This command sets up the necessary environment for Autoware with required volumes mounted.

2. Source the setup file:

    ```bash
    source ~/autoware/install/setup.bash
    ```

   This ensures that the Autoware setup is sourced in the current terminal.

3. Launch Autoware using the following command, adjusting parameters as needed:

    ```bash
    ros2 launch autoware_launch planning_simulator.launch.xml map_path:=$HOME/autoware_map/sample-map-planning vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
    ```

   This command launches Autoware with the specified parameters for planning and simulation.
   
---

## Ackermann Control Command ROS2 Topic

### Overview
This section provides information about the ROS2 topic `/control/command/control_cmd`, which publishes messages of type `autoware_auto_control_msgs/msg/AckermannControlCommand`. This topic is used to convey control commands, particularly for Ackermann-type vehicles.

### Sample Output
Here is a snippet of the sample output from the `/control/command/control_cmd` topic:

```yaml
---
stamp:
  sec: 1706765335
  nanosec: 172912518
lateral:
  stamp:
    sec: 1706765335
    nanosec: 172714015
  steering_tire_angle: -0.0029230378568172455
  steering_tire_rotation_rate: -0.0016902522183954716
longitudinal:
  stamp:
    sec: 1706765335
    nanosec: 172788226
  speed: 1.659965991973877
  acceleration: -0.3646732568740845
  jerk: 0.0
---
...
```

### Explanation

- **`stamp`**: Contains the timestamp indicating when the control command was generated. It includes seconds (`sec`) and nanoseconds (`nanosec`).

- **`lateral`**: Information related to lateral control.
  - `steering_tire_angle`: The angle of the steering tire, indicating the desired direction.
  - `steering_tire_rotation_rate`: The rotation rate of the steering tire.

- **`longitudinal`**: Information related to longitudinal control.
  - `speed`: The speed of the vehicle in meters per second.
  - `acceleration`: The acceleration of the vehicle.
  - `jerk`: The jerk (rate of change of acceleration) of the vehicle.

---

## TF Topic - `tf2_msgs/msg/TFMessage`

### Sample Output
Here is a snippet of the sample output from the `/tf` topic:

```yaml
transforms:
- header:
    stamp:
      sec: 1706766702
      nanosec: 131340168
    frame_id: map
  child_frame_id: base_link
  transform:
    translation:
      x: 3722.6949928615045
      y: 73749.92152812779
      z: 19.483417879617267
    rotation:
      x: 0.0
      y: -0.0
      z: -0.8444251767627481
      w: -0.5356735207654021
---
```

### Explanation

- **`transforms`**: Contains information about the transformations.
  - **`header`**: Includes timestamp, frame_id, and child_frame_id.
    - `stamp`: Timestamp with seconds (`sec`) and nanoseconds (`nanosec`).
    - `frame_id`: The reference frame.
    - `child_frame_id`: The child frame.
  - **`transform`**: Describes the translation and rotation of the child frame relative to the reference frame.
    - **`translation`**: The translation along the x, y, and z axes.
    - **`rotation`**: The rotation represented by quaternion (x, y, z, w).

This data represents a transformation from the "map" frame to the "base_link" frame at a specific timestamp. It is essential for understanding the spatial relationships between different frames in the robotic system.

---


