from setuptools import find_packages
from setuptools import setup

package_name = "ros2_node_manager_cli"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    install_requires=["ros2cli", "ros2node"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    zip_safe=True,
    author="Hussein Kabbout",
    author_email="kabbohus@students.zhaw.ch",
    maintainer="Hussein Kabbout",
    maintainer_email="kabbohus@students.zhaw.ch",
    url="https://github.com/BA23-Robotic-Fleet-Management/ros2_node_manager",
    download_url="https://github.com/BA23-Robotic-Fleet-Management/ros2_node_manager/releases",
    keywords=[],
    classifiers=[],
    description="CLI with which ROS2 nodes that are running on a robot can be started / stopped from anywhere",
    license="Apache License, Version 2.0",
    tests_require=["pytest"],
    entry_points={
        "ros2node.verb": [
            "stop = src.verb.stop:StopNodeVerb",
            "start = src.verb.start:StartNodeVerb",
        ],
    },
)
