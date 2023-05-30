# ROS2 node manager

This repository contains all components of the ROS2 node manager.

## Requirements

- ros2 humble
- rust + cargo (https://www.rust-lang.org/tools/install)
- ros2_rust (https://github.com/ros2-rust/ros2_rust#sounds-great-how-can-i-try-this-out)

## Build

```bash
cd ros2-node-manager/
source /opt/ros/humble/setup.bash
# This command assumes that ros2_rust was installed in ~/ros2_rust
# and not in a directory called `workspace` like in the linked documentation
source ~/ros2_rust/install/setup.bash
colcon build --parallel-workers 4 --symlink-install
```

## How to use

### CLI

The CLI tools are directly integrated in `ros2cli`.

List nodes:

```bash
source ros2-node-manager/install/setup.bash
ros2 node-manager list-nodes ROBOTER_NAME
```

Start node:

```bash
source ros2-node-manager/install/setup.bash
ros2 node-manager start ROBOTER_NAME NODE_NAME
```

Stop node:

```bash
source ros2-node-manager/install/setup.bash
ros2 node-manager stop ROBOTER_NAME NODE_NAME
```

The optional arguments `--stop_time`, when stopping a node, or `--start_time`, when starging a node, can be defined
to extend the time used to check if the node could be started or stopped successfully.

### Node manager server

The node manager server lives on each robot and controls nodes via systemd unit files.
An [example unit file](./server/misc/free-fleet-server.service) can be found in [misc](./server/misc).
All unit files should be stored in `~/.local/share/systemd/user/`.
If you want your unit file to be found by the `list-nodes` command, then you also have to
enable the systemd unit with `systemctl --user enable UNIT_NAME`.
This will install the systemd unit in the target `ros2-node-manager`.

Start server:

```bash
source ros2-node-manager/install/setup.bash
ros2 run ros2_node_manager_server ros2_node_manager_server --robot-name ROBOTER_NAME
```
