import rclpy
from src.api import NodeManagerClient
from src.verb import VerbExtension


class StopNodeVerb(VerbExtension):
    """Stop a node by name"""

    def add_arguments(self, parser, cli_name):
        parser.add_argument("robot_name", help="Name of the robot")
        parser.add_argument(
            "node_name", help="Name of the ROS2 node that you want to stop"
        )
        parser.add_argument(
            "--stop_time",
            required=False,
            help="Number of seconds to wait until the node is fully stopped. Defaults to 1 second.",
            default=1,
            type=int,
        )
        # TODO: Add auto completion

    def main(self, *, args):
        rclpy.init()
        client = NodeManagerClient()
        result = client.stop_node(args.robot_name, args.node_name, args.stop_time)

        if result.success:
            print(f"Node {args.node_name} on {args.robot_name} has been stopped")
        else:
            print(f"Could not stop {args.node_name}")
