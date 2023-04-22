from ros2_node_manager_interfaces.srv import StopNode
from ros2_node_manager_interfaces.srv import StartNode
from ros2_node_manager_interfaces.srv import ListNodes
import rclpy
from rclpy.node import Node


class NodeManagerClient(Node):
    def __init__(self):
        super().__init__("ros2_node_manager_cli")

    def stop_node(self, robot_name, node_name, stop_time):
        client = self.__client_connect(StopNode, f"{robot_name}_stop_node")

        request = StopNode.Request()
        request.node_name = node_name
        request.stop_time = stop_time

        return self.__send_request(client, request)

    def start_node(self, robot_name, node_name, start_time):

        client = self.__client_connect(StartNode, f"{robot_name}_start_node")

        request = StartNode.Request()
        request.node_name = node_name
        request.start_time = start_time

        return self.__send_request(client, request)

    def list_nodes(self, robot_name):
        client = self.__client_connect(ListNodes, f"{robot_name}_list_nodes")
        return self.__send_request(client, ListNodes.Request())

    def __client_connect(self, request_type, client_name):
        client = self.create_client(request_type, client_name)
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"Trying to connect to service...")

        return client

    def __send_request(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
