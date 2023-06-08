# ROS2 Node Manager

This repository contains all the components of the ROS2 node manager.
The node manager allows users to centrally manage the state of individual nodes on the remote fleet.
This could later on be used to shut down nodes that are not currently required, for example the image recognition software during charging.

## Requirements

- ROS 2 humble
- Rust and cargo (https://www.rust-lang.org/tools/install)
- `ros2_rust` installed to `~/ros2_rust` (https://github.com/ros2-rust/ros2_rust#sounds-great-how-can-i-try-this-out)

## Build

```bash
cd ros2-node-manager/
source /opt/ros/humble/setup.bash
source ~/ros2_rust/install/setup.bash
colcon build --parallel-workers 4 --symlink-install
```

## How to use

### CLI

The CLI tools are directly integrated in the `ros2cli`.

List all managed nodes, including their state:

```bash
source ros2-node-manager/install/setup.bash
ros2 node-manager list-nodes ROBOT_NAME
```

Start a node:

```bash
source ros2-node-manager/install/setup.bash
ros2 node-manager start ROBOT_NAME NODE_NAME
```

Stop a node:

```bash
source ros2-node-manager/install/setup.bash
ros2 node-manager stop ROBOT_NAME NODE_NAME
```

The optional argument `--start_time` (or the respective `--stop_time`) sets the timeout after which the state of the node is checked.

### Node Manager Server

The node manager server lives on each robot and controls the ROS 2nodes using systemd unit files.
An example unit file can be found in [misc](./server/misc).
All unit files should be stored in `~/.local/share/systemd/user/`.
If you want your unit file to be found by the `list-nodes` command, then you must enable the systemd unit with `systemctl --user enable UNIT_NAME`.
This will install the systemd unit in the `ros2-node-manager` target.

Start the server:

```bash
source ros2-node-manager/install/setup.bash
ros2 run ros2_node_manager_server ros2_node_manager_server --robot-name ROBOT_NAME
```
