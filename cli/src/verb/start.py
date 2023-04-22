import rclpy
from src.api import NodeManagerClient
from src.verb import VerbExtension


class StartNodeVerb(VerbExtension):
    """Start a node by name"""

    def add_arguments(self, parser, cli_name):
        """
        Specify arguments for the start verb
        """
        parser.add_argument("robot_name", help="Name of the robot")
        parser.add_argument(
            "node_name", help="Name of the ROS2 node that you want to start"
        )
        parser.add_argument(
            "--start_time",
            help="Number of seconds to wait until the node is started correctly. Defaults to 1 second.",
            default=1,
            type=int,
        )
        # TODO: Add auto completion

    def main(self, *, args):
        """
        Main
        """
        rclpy.init()
        client = NodeManagerClient()
        result = client.start_node(args.robot_name, args.node_name, args.start_time)

        if result.success:
            print(f"Node {args.node_name} on {args.robot_name} has been started")
        else:
            print(f"Could not start {args.node_name}")
