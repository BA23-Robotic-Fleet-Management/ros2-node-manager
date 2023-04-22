import rclpy
from src.api import NodeManagerClient
from src.verb import VerbExtension


class ListNodesVerb(VerbExtension):
    """List all nodes that are managed by the node manager"""

    def add_arguments(self, parser, cli_name):
        """
        Specify arguments for the list verb
        """
        parser.add_argument("robot_name", help="Name of the robot")

    def main(self, *, args):
        """
        Main
        """
        rclpy.init()
        client = NodeManagerClient()
        print(client.list_nodes(args.robot_name).nodes)
