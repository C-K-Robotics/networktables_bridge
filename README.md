# networktables_bridge

## Overview

The `networktables_bridge` package is designed to establish an interface between the FRC NetworkTables (specifically NT4, with potential support for NT3). It facilitates communication by subscribing to NetworkTable topics and publishing them to ROS 2 topics under the `/networktable` namespace. Additionally, it can subscribe to ROS 2 topics and publish them to NetworkTable topics without a specific prefix, allowing for complete customization.

## Features

- **NetworkTable to ROS 2:** Automatically or manually subscribe to NetworkTable topics and publish them as ROS 2 topics with a prefix namespace `/networktable`.
- **ROS 2 to NetworkTable:** Subscribe to ROS 2 topics and publish them to NetworkTable topics with customizable settings, avoiding unnecessary bandwidth consumption.

## Installation

### Prerequisites

- **RobotPy**: Required for interfacing with NetworkTables. Install it via pip:
  ```bash
  pip3 install robotpy
  ```
  Refer to the [RobotPy setup guide](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/python-setup.html) for more details.

### Supported Platforms
- **Windows**: Version 10 & 11, 64-bit only.
- **macOS**: Version 12 or higher.
- **Linux**: Ubuntu 22.04, 64-bit. Other distributions with glibc >= 2.35 may work but are unsupported.

### Python Compatibility
Python versions supported: 3.8, 3.9, 3.10, 3.11, and 3.12 (only version available for roboRIO).

### ROS Compatibility
The `networktables_bridge` package is compatible with the following ROS distributions:
- **ROS 2 Humble**: Recommended for new  projects running on Ubuntu 22.04.
- **ROS 2 Iron**: Supported for users requiring newer features introduced in Iron.
- **ROS 2 Foxy**: Compatible with Ubuntu 20.04; however, note that support for Foxy ended in May 2023.
- **Potential ROS Noetic Support**: Development efforts may be made to support ROS Noetic (ROS 1) upon request.

### Steps to Install
1. Clone the repository into your workspace's src directory:
```bash
git clone <repository-url> {workspace}/src/networktables_bridge
```
2. Specify the NT server IP address in the YAML files:
   - For `nt_client_pub_node`, edit `nt_client_pub.yaml`:
        ```yml
        NT_Client_Publish_Node:
        ros__parameters:
            NT_server_ip: "10.80.20.2"
            sampling_time: 0.1
            
            sub_rostopic_names: [""]  # the ROS topics you want to subscribe, like "/test_cmd_vel", "/test/test_pose"
            msg_types: [""] # the corresponding ROS msg type, like "geometry_msgs/msg/Twist", "geometry_msgs/msg/Pose"
            pub_NT_names: [""]  # the NetworkTable name to be published to NT, like "/data/test_vel","/data/data/test_pose"
        ```
    - For `nt_client_sub_node`, edit `nt_client_sub.yaml`:
        ```yml
        NT_Client_Subscribe_Node:
        ros__parameters:
            NT_server_ip: "10.80.20.2"
            sampling_time: 0.1
            
            sub_NT_names: ['']  # the NT names you want to subscribe, like "/data/test_vel"
            msg_types: [''] # the corresponding ROS msg type, like "geometry_msgs/msg/Twist"
            pub_rostopic_names: ['']  # the ROS topic name to be published, like "/restored_cmd_vel"
            automated: True # Default: True
        ```
3. Use `rosdep` to install dependencies, build your workspace with `colcon`, and source your workspace:
```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```
4. Execution:
    - To run an individual node:
        ```bash
        ros2 run networktables_bridge nt_client_pub_node --ros-args --params-file nt_client_pub.yaml
        ros2 run networktables_bridge nt_client_sub_node --ros-args --params-file nt_client_sub.yaml
        ```
    - Use a launch file if you need both nodes running simultaneously:
        ```bash
        ros2 launch networktables_bridge nt_client.launch.py
        ```

## License
This project is licensed under the Apache License, Version 2.0, January 2004.

## Contribution
Contributions are welcome! Please follow the standard GitHub fork-and-pull request workflow. For more detailed documentation on developing and extending this package, refer to the documentation files included in this repository.