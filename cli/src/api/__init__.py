from ros2_node_manager_interfaces.srv import StopNode
from ros2_node_manager_interfaces.srv import StartNode
from ros2_node_manager_interfaces.srv import ListNodes
import rclpy
from rclpy.node import Node


class NodeManagerClient(Node):
    """
    Client class which can be used to interact with the node manager
    """

    def __init__(self):
        """Constructor"""
        super().__init__("ros2_node_manager_cli")

    def stop_node(self, robot_name, node_name, stop_time):
        """
        Stop a node on the robot by calling the StopNode service.

        Args:
            robot_name (str): Name of the robot on which we want to stop a node
            node_name (str): Name of the node to stop
            stop_time (int): Time in seconds to wait for the service to be fully stopped

        Returns:
            (StopNode_Response): Response from the server
        """
        client = self.__client_connect(StopNode, f"{robot_name}_stop_node")

        request = StopNode.Request()
        request.node_name = node_name
        request.stop_time = stop_time

        return self.__send_request(client, request)

    def start_node(self, robot_name, node_name, start_time):
        """
        Start a node on the robot by calling the StartNode service.

        Args:
            robot_name (str): Name of the robot on which we want to start a node
            node_name (str): Name of the node to start
            stop_time (int): Time in seconds to wait for the service to be started completly

        Returns:
            (StartNode_Response): Response from the server
        """

        client = self.__client_connect(StartNode, f"{robot_name}_start_node")

        request = StartNode.Request()
        request.node_name = node_name
        request.start_time = start_time

        return self.__send_request(client, request)

    def list_nodes(self, robot_name):
        """
        List all nodes on the robot that are managed by the node manager

        Args:
            robot_name (str): Name of the robot on which we want to list all nodes

        Returns:
            (ListNodes_Response): Response from the server
        """
        client = self.__client_connect(ListNodes, f"{robot_name}_list_nodes")
        return self.__send_request(client, ListNodes.Request())

    def __client_connect(self, request_type, service_name):
        """
        Connect to a ROS2 service

        Args:
            request_type (Interface): Type of the service message
            service_name (str): Name of the service to connect to

        Returns:
            (rclpy.Client): Client object that was created
        """
        client = self.create_client(request_type, service_name)
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"Trying to connect to service...")

        return client

    def __send_request(self, client, request):
        """
        Call a ROS2 service with a specific request

        Args:
            client (Client): Client to use for the call
            service_name (str): Name of the service to call

        Returns:
            (Interface_Reponse): Reponse from the server
        """
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
