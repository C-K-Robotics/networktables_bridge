# networktables_bridge

## Overview

`networktables_bridge` provides a ROS 2 lifecycle node that bridges selected NetworkTables (NT4) keys into ROS 2 topics. The current implementation is a C++ lifecycle node that listens to AdvantageKit/SystemStats topics on NetworkTables and publishes a `frc_msgs/msg/MiscReport` message on the `misc_report` topic.

This README focuses on the C++ bridge present in this package (executable: `nt_bridge_node`).

## What this node does

- Connects to a NetworkTables server (typically a roboRIO or NT server) as an NT4 client.
- Subscribes to AdvantageKit/SystemStats and NTClients keys and converts them into ROS 2 `frc_msgs/msg/MiscReport` and `frc_msgs/msg/NTClient` messages.
- Publishes `misc_report` (type `frc_msgs/msg/MiscReport`).

Note: the node implements the ROS 2 managed lifecycle (rclcpp_lifecycle). After launching, you must transition the node to the `activate` state for it to start publishing.

## Build / Install

Prerequisites
- ROS 2 (tested with Humble / Iron)
- Colcon build tool and `rosdep`
- System dependencies for WPILib/ntcore when building the node (the CMake file links against `ntcore`, `wpinet`, `wpiutil`, `wpimath`). Ensure your environment provides these libraries if you need NT4 integration.

Typical steps

```bash
# from your ROS 2 workspace root
git clone <repo-url> src/networktables_bridge
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select networktables_bridge
source install/setup.bash
```

If your environment is missing WPILib/ntcore libraries, follow WPILib/ntcore installation instructions for your platform or build those packages into the workspace.

## Launching

This package provides a launch file: `launch/nt_bridge.launch.py`.

Launch with the default (localhost) NetworkTables server:

```bash
ros2 launch networktables_bridge nt_bridge.launch.py
```

Provide a different NetworkTables server IP (e.g., roboRIO at 10.80.20.2):

```bash
ros2 launch networktables_bridge nt_bridge.launch.py nt_server_ip:=10.80.20.2
```

You can also run the executable directly (useful for debugging):

```bash
ros2 run networktables_bridge nt_bridge_node --ros-args -p nt_server_ip:=10.80.20.2
```

Lifecycle notes
- The node is a lifecycle node and will not publish until it is transitioned into the `active` state. Use the ROS 2 lifecycle CLI to transition the node:

```bash
ros2 lifecycle set /nt_bridge_node configure
ros2 lifecycle set /nt_bridge_node activate
```

After activation the node will publish `misc_report` periodically (50 Hz step timer logic inside the node).

## Parameters

- `nt_server_ip` (string, default: `127.0.0.1`) — IP address of the NetworkTables server.
- `nt_remote_id` (string, default: node name) — remote id used when starting the NT client.

These are set in the launch file by default; pass different values via the launch arguments or `--ros-args -p` when running the executable.

## Topics and messages

- Publishes: `misc_report` — type: `frc_msgs/msg/MiscReport` (contains team number, battery values, various rail voltages/currents, NT client list, timestamp, etc.)

The node maps values from AdvantageKit/SystemStats and NTClients NetworkTables keys into the fields of `MiscReport` and `NTClient` messages.

## Development notes

- Executable: `nt_bridge_node` (defined in `CMakeLists.txt`).
- The node uses `networktables_common` helpers (see `networktables_common/nt_pubsub.hpp`) to subscribe to NT topics.
- CMake links against `ntcore`, `wpinet`, `wpiutil`, and `wpimath` — these must be available at link time.

## License
This package is licensed under LGPLv3 (see `package.xml`).

## Contributing
Contributions welcome — please open issues or PRs with changes. For build or runtime problems, include logs and platform details.
