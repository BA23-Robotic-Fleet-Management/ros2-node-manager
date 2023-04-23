use std::process::Command;
use std::thread;
use std::time::Duration;

use dbus;
use dbus::arg::PropMap;
use dbus::arg::RefArg;
use dbus::blocking::SyncConnection;
use dbus::Path;
use std::sync::Arc;

static NODE_MANAGER_TARGET_UNIT: &str = "ros2-node-manager.target";

static DBUS_SYSTEMD_DESTINATION: &str = "org.freedesktop.systemd1";
static DBUS_SYSTEMD_OBJECT_PATH: &str = "/org/freedesktop/systemd1";
static DBUS_SYSTEMD_MANAGER_INTERFACE: &str = "org.freedesktop.systemd1.Manager";

// Start a systemd unit and return whether the operation was successfull
// pub fn start_unit(name: String, start_time: u64) -> bool {
//     let mut success = false;

//     Command::new("systemctl")
//         .arg("--user")
//         .arg("start")
//         .arg(&name)
//         .spawn();

//     // Wait until process is stopped
//     thread::sleep(Duration::from_secs(start_time));

//     let result = Command::new("systemctl")
//         .arg("--user")
//         .arg("is-active")
//         .arg(&name)
//         .status();

//     if let Ok(exit_status) = result {
//         success = exit_status.success();
//     }

//     success
// }

// Stop a systemd unit and return whether the operation was successfull
pub fn stop_unit(name: String, stop_time: u64) -> bool {
    let mut success = false;

    Command::new("systemctl")
        .arg("--user")
        .arg("stop")
        .arg(&name)
        .spawn();

    // Wait until process is stopped
    thread::sleep(Duration::from_secs(stop_time));

    let result = Command::new("systemctl")
        .arg("--user")
        .arg("is-active")
        .arg(&name)
        .output();

    if let Ok(output) = result {
        success = output.stdout == "inactive\n".as_bytes().to_vec();
    }

    success
}

// Get a list of all nodes that are controlled by the node manager
// The list is returned as a String
pub fn list_nodes() -> String {
    if let Ok(output) = Command::new("systemctl")
        // Activate colors
        .env("SYSTEMD_COLORS", "1")
        .arg("--user")
        .arg("list-dependencies")
        .arg(NODE_MANAGER_TARGET_UNIT)
        .output()
    {
        // First line will always be the target name, we don't want to show that to the user
        // so we remove it
        // The index of the first line is calculated by getting the index of the first line ending
        // (LF). Because the output is in bytes we search for the u8 10, which is LF in utf-8
        let index_first_line = output.stdout.iter().position(|&r| r == 10).unwrap();
        // Skip the first line by creating a slice from the Vec
        String::from_utf8(output.stdout[index_first_line + 1..].to_vec())
            .unwrap_or(String::new())
            // Remove stuff that we don't want in the output
            .replace("└─", "")
            .replace(".service", "")
            // Remove any trailing whitespace / new line
            .trim()
            .to_string()
    } else {
        String::new()
    }
}

pub fn start_unit<T: for<'z> dbus::arg::Get<'z> + dbus::arg::Arg + std::fmt::Debug>(
    connection: Arc<SyncConnection>,
    name: String,
) -> bool {
    let result = connection
        .with_proxy(
            DBUS_SYSTEMD_DESTINATION,
            Path::new(DBUS_SYSTEMD_OBJECT_PATH).unwrap(),
            Duration::new(5, 0),
        )
        // See: https://www.freedesktop.org/software/systemd/man/org.freedesktop.systemd1.html#Methods
        // for more infomation on "StartUnit"
        .method_call(
            DBUS_SYSTEMD_MANAGER_INTERFACE,
            "StartUnit",
            (name + ".service", "replace"),
        )
        .map(|r: (T,)| r.0);

    println!("{:?}", result);

    false
}
