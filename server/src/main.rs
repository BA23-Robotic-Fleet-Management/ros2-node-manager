use std::env;

use anyhow::{Error, Result};

fn handle_stop_node(
    _request_header: &rclrs::rmw_request_id_t,
    request: ros2_node_manager_interfaces::srv::StopNode_Request,
) -> ros2_node_manager_interfaces::srv::StopNode_Response {
    println!("Someone requested to stop: {}", request.node_name);
    ros2_node_manager_interfaces::srv::StopNode_Response {
        success: true,
    }
}

fn main() -> Result<(), Error> {
    let context = rclrs::Context::new(env::args())?;
    let mut node = rclrs::create_node(&context, "ros2_node_manager_server")?;
    let stop_node_service = node
        .create_service::<ros2_node_manager_interfaces::srv::StopNode, _>("stop_node", handle_stop_node)?;

    println!("Starting server");
    rclrs::spin(&node).map_err(|err| err.into())
}
