mod systemd;

use anyhow::{Error, Result};
use clap::Parser;
use ros2_node_manager_interfaces::srv::{ListNodes, ListNodes_Request, ListNodes_Response};
use ros2_node_manager_interfaces::srv::{StartNode, StartNode_Request, StartNode_Response};
use ros2_node_manager_interfaces::srv::{StopNode, StopNode_Request, StopNode_Response};
use std::env;

// Handler for a StopNode request
fn handle_stop_node(
    _request_header: &rclrs::rmw_request_id_t,
    request: StopNode_Request,
) -> StopNode_Response {
    log::info!("Someone requested to stop: {}", request.node_name);
    StopNode_Response {
        success: systemd::stop_unit(request.node_name, request.stop_time),
    }
}

// Handler for a StartNode request
fn handle_start_node(
    _request_header: &rclrs::rmw_request_id_t,
    request: StartNode_Request,
) -> StartNode_Response {
    log::info!("Someone requested to start: {}", request.node_name);
    StartNode_Response {
        success: systemd::start_unit(request.node_name, request.start_time),
    }
}

// Handler for a ListNodes request
fn handle_list_nodes(
    _request_header: &rclrs::rmw_request_id_t,
    _request: ListNodes_Request,
) -> ListNodes_Response {
    ListNodes_Response {
        nodes: systemd::list_nodes(),
    }
}

// Definition of the command line arguments
#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    #[arg(long)]
    robot_name: String,
}

// Main function
fn main() -> Result<(), Error> {
    // Init logger
    env_logger::init_from_env(env_logger::Env::new().default_filter_or("info"));
    // Parse command line arguments
    let args = Args::parse();
    // Init ros2 node
    let context = rclrs::Context::new(env::args())?;
    let mut node = rclrs::create_node(
        &context,
        &format!("{}_node_manager_server", args.robot_name),
    )?;
    // Register stop node service
    let _stop_node_service = node.create_service::<StopNode, _>(
        &format!("{}_stop_node", args.robot_name),
        handle_stop_node,
    )?;
    // Register start node service
    let _start_node_service = node.create_service::<StartNode, _>(
        &format!("{}_start_node", args.robot_name),
        handle_start_node,
    )?;
    // Register list nodes service
    let _list_nodes_service = node.create_service::<ListNodes, _>(
        &format!("{}_list_nodes", args.robot_name),
        handle_list_nodes,
    )?;
    // Start server
    log::info!("Starting server");
    rclrs::spin(&node).map_err(|err| err.into())
}
