from ros2_node_manager_interfaces.srv import StopNode
import rclpy
from rclpy.node import Node


class NodeManagerClient(Node):

    def __init__(self):
        super().__init__('ros2_node_manager_client')
        self.client = self.create_client(StopNode, 'stop_node')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Trying to connect to service StopNode...")
        self.request = StopNode.Request()

    def stop_node(self, node_name):
        self.request.node_name = node_name
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

