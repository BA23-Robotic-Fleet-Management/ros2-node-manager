use std::env;
use ros2_node_manager_interfaces::srv::{StopNode,StopNode_Request,StopNode_Response};
use ros2_node_manager_interfaces::srv::{StartNode,StartNode_Request,StartNode_Response};
use anyhow::{Error, Result};

fn handle_stop_node(
    _request_header: &rclrs::rmw_request_id_t,
    request: StopNode_Request,
) -> StopNode_Response {
    println!("Someone requested to stop: {}", request.node_name);
    StopNode_Response {
        success: true,
    }
}

fn handle_start_node(
    _request_header: &rclrs::rmw_request_id_t,
    request: StartNode_Request,
) -> StartNode_Response {
    println!("Someone requested to start: {}", request.node_name);
    StartNode_Response {
        success: true,
    }
}

fn main() -> Result<(), Error> {
    let context = rclrs::Context::new(env::args())?;
    let mut node = rclrs::create_node(&context, "ros2_node_manager_server")?;
    // Register stop node service
    let stop_node_service = node
        .create_service::<StopNode, _>("stop_node", handle_stop_node)?;
    // Register start node service
    let start_node_service = node
        .create_service::<StartNode, _>("start_node", handle_start_node)?;

    println!("Starting server");
    rclrs::spin(&node).map_err(|err| err.into())
}
