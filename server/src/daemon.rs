use daemonize::Daemonize;
use log;
use std::fs::File;

pub struct NodeManagerDaemon {}

impl NodeManagerDaemon {
    pub fn start() {
        let stdout = File::create("/tmp/daemon.log").unwrap();
        let stderr = File::create("/tmp/daemon_error.log").unwrap();

        let daemonize = Daemonize::new()
            .pid_file("/tmp/ros2-node-manager.pid")
            .chown_pid_file(true)
            .working_directory("/tmp")
            .user("roscon")
            .group("roscon")
            .stdout(stdout)
            .stderr(stderr);

        match daemonize.start() {
            Ok(_) => log::info!("ROS2 node manager has been successfully daemonnized!"),
            Err(e) => log::error!("Error, {}", e),
        }
    }
}
