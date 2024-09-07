from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():  # all launch files need a function with this name
    # LaunchConfiguration is just a "parameter" that can be passed from the cmd line
    compass_type = LaunchConfiguration("compass_type")
    gps_type = LaunchConfiguration("gps_type")

    # give parameters default values
    compass_arg = DeclareLaunchArgument(
        "compass_type", default_value="actual", choices=["actual", "fake"]
    )
    gps_arg = DeclareLaunchArgument(
        "gps_type", default_value="actual", choices=["actual", "fake"]
    )

    # create nodes
    # can't use the value in the parameter until we return the launch description.
    # thus, the lists, allow us to say "substitute `compass_type` with the actual value at runtime"
    gps_node = Node(
        package="compass",
        executable=[compass_type, "_compass"],
    )
    compass_node = Node(package="gps", executable=[gps_type, "_gps"])

    return LaunchDescription([compass_arg, gps_arg, gps_node, compass_node])
