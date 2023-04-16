# ROS2 node manager

This repository contains all components of the ROS2 node manager.

## Requirements

- ros2 humble
- rust + cargo (https://www.rust-lang.org/tools/install)
- ros2_rust (https://github.com/ros2-rust/ros2_rust#sounds-great-how-can-i-try-this-out)

## Build

```
cd ros2_node_manager/
source /opt/ros/humble/setup.bash
source ~/ros2_rust/install/setup.bash
colcon build --parallel-workers 4 --symlink-install
```

## How to use

### CLI

The CLI tools is directly integrated in `ros2cli`.

Stop node:

```
source ros2_node_manager/install/setup.bash
ros2 node stop NODE_NAME
```

### Node manager server

The node manager server lives on each robot and controls nodes via systemd unit files.

Start server:

```
source ros2_node_manager/install/setup.bash
ros2 run ros2_node_manager_server ros2_node_manager_server
```


