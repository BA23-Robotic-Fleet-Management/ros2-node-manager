import rclpy
from ros2cli.verb import VerbExtension
from ros2node.api import NodeNameCompleter
from src.api import NodeManagerClient


class StopNodeVerb(VerbExtension):
    def add_arguments(self, parser, cli_name):
        node_name_argument = parser.add_argument(
            "node_name", help="Name of the ROS2 node that you want to stop"
        )
        node_name_argument.completer = NodeNameCompleter()

    def main(self, *, args):
        rclpy.init()
        client  = NodeManagerClient()
        result = client.stop_node(args.node_name)

        if result.success:
            print(f"Node {args.node_name} has been stopped")
        else:
            print(f"Could not stop {args.node_name}")
