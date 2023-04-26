from setuptools import find_packages
from setuptools import setup

package_name = "ros2_node_manager_cli"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    install_requires=["ros2cli"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    zip_safe=True,
    author="Hussein Kabbout",
    author_email="hussein.kabbout@hotmail.com",
    maintainer="Hussein Kabbout",
    maintainer_email="hussein.kabbout@hotmail.com",
    url="https://github.com/BA23-Robotic-Fleet-Management/ros2-node-manager",
    download_url="https://github.com/BA23-Robotic-Fleet-Management/ros2-node-manager/releases",
    keywords=[],
    classifiers=[],
    description="CLI with which ROS2 nodes that are running on a robot can be started / stopped from anywhere",
    license="Apache License, Version 2.0",
    tests_require=["pytest"],
    entry_points={
        "ros2cli.command": [
            "node-manager = src.command.manager:NodeManagerCommand",
        ],
        "ros2cli.extension_point": [
            "src.verb = src.verb:VerbExtension",
        ],
        "ros2_node_manager_cli.verb": [
            "stop = src.verb.stop:StopNodeVerb",
            "start = src.verb.start:StartNodeVerb",
            "list-nodes = src.verb.list:ListNodesVerb",
        ],
    },
)
