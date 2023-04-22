use std::process::Command;
use std::thread;
use std::time::Duration;

static NODE_MANAGER_TARGET_UNIT: &str = "ros2-node-manager.target";

pub fn start_unit(name: String, start_time: u64) -> bool {
    let mut success = false;

    Command::new("systemctl")
        .arg("--user")
        .arg("start")
        .arg(&name)
        .spawn();

    // Wait until process is stopped
    thread::sleep(Duration::from_secs(start_time));

    let result = Command::new("systemctl")
        .arg("--user")
        .arg("is-active")
        .arg(&name)
        .status();

    if let Ok(exit_status) = result {
        success = exit_status.success();
    }

    success
}

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
            // Remove wierd charecter that we don't want in the output
            .replace("└─", "")
            // Remove any trailing whitespace / new line
            .trim()
            .to_string()
    } else {
        String::new()
    }
}
