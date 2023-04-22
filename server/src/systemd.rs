use std::process::Command;
use std::thread;
use std::time::Duration;

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
