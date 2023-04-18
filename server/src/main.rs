use anyhow::{Error, Result};
use clap::Parser;
use ros2_node_manager_interfaces::srv::{StartNode, StartNode_Request, StartNode_Response};
use ros2_node_manager_interfaces::srv::{StopNode, StopNode_Request, StopNode_Response};
use std::env;

fn handle_stop_node(
    _request_header: &rclrs::rmw_request_id_t,
    request: StopNode_Request,
) -> StopNode_Response {
    println!("Someone requested to stop: {}", request.node_name);
    StopNode_Response { success: true }
}

fn handle_start_node(
    _request_header: &rclrs::rmw_request_id_t,
    request: StartNode_Request,
) -> StartNode_Response {
    println!("Someone requested to start: {}", request.node_name);
    StartNode_Response { success: true }
}

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    #[arg(long)]
    robot_name: String,
}

fn main() -> Result<(), Error> {
    let args = Args::parse();

    let context = rclrs::Context::new(env::args())?;
    let mut node = rclrs::create_node(
        &context,
        &format!("{}_node_manager_server", args.robot_name),
    )?;
    // Register stop node service
    let stop_node_service = node.create_service::<StopNode, _>(
        &format!("{}_stop_node", args.robot_name),
        handle_stop_node,
    )?;
    // Register start node service
    let start_node_service = node.create_service::<StartNode, _>(
        &format!("{}_start_node", args.robot_name),
        handle_start_node,
    )?;

    println!("Starting server");
    rclrs::spin(&node).map_err(|err| err.into())
}
