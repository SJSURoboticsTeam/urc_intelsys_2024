from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # all launch files must have a function with this name

    return LaunchDescription(
        # put a list of nodes to be launched here
        [
            Node(package="urc_intelsys_2024", executable="fake_gps"),
        ]
    )
