from ros2_node_manager_interfaces.srv import StopNode
from ros2_node_manager_interfaces.srv import StartNode
import rclpy
from rclpy.node import Node


class NodeManagerClient(Node):
    def __init__(self):
        super().__init__("ros2_node_manager_cli")

    def stop_node(self, robot_name,  node_name):
        client = self.create_client(StopNode, f"{robot_name}_stop_node")
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Trying to connect to service StopNode...")

        request = StopNode.Request()
        request.node_name = node_name

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


    def start_node(self, robot_name,  node_name):
        client = self.create_client(StartNode, f"{robot_name}_start_node")
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Trying to connect to service StartNode...")

        request = StartNode.Request()
        request.node_name = node_name

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

