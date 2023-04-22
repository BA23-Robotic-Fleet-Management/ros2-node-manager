# ROS2 node manager

This repository contains all components of the ROS2 node manager.

## Requirements

- ros2 humble
- rust + cargo (https://www.rust-lang.org/tools/install)
- ros2_rust (https://github.com/ros2-rust/ros2_rust#sounds-great-how-can-i-try-this-out)

## Build

```
cd ros2-node-manager/
source /opt/ros/humble/setup.bash
source ~/ros2_rust/install/setup.bash
colcon build --parallel-workers 4 --symlink-install
```

## How to use

### CLI

The CLI tools is directly integrated in `ros2cli`.

Start node:

```
source ros2-node-manager/install/setup.bash
ros2 node-manager start ROBOTER_NAME NODE_NAME
```

Stop node:

```
source ros2-node-manager/install/setup.bash
ros2 node-manager stop ROBOTER_NAME NODE_NAME
```

The optional arguments `--stop_time`, when stopping a node, or `--start_time`, when starging a node, can be defined
to extend the time used to check if the node could be started or stopped successfully.

### Node manager server

The node manager server lives on each robot and controls nodes via systemd unit files.

Start server:

```
source ros2-node-manager/install/setup.bash
ros2 run ros2_node_manager_server ros2_node_manager_server --robot-name ROBOTER_NAME
```


