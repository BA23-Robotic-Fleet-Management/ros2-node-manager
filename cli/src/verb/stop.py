import rclpy
from src.api import NodeManagerClient
from src.verb import VerbExtension


class StopNodeVerb(VerbExtension):
    """Stop a node by name"""

    def add_arguments(self, parser, cli_name):
        robot_name_argument = parser.add_argument(
            "robot_name", help="Name of the robot"
        )
        node_name_argument = parser.add_argument(
            "node_name", help="Name of the ROS2 node that you want to stop"
        )
        # TODO: Add auto completion

    def main(self, *, args):
        rclpy.init()
        client = NodeManagerClient()
        result = client.stop_node(args.robot_name, args.node_name)

        if result.success:
            print(f"Node {args.node_name} on {args.robot_name} has been stopped")
        else:
            print(f"Could not stop {args.node_name}")
