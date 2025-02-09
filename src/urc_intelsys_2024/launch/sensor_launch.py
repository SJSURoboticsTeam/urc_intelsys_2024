from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():  # all launch files need a function with this name
    # get parameters
    config = get_package_share_directory("urc_intelsys_2024") + "/config/config.yaml"

    # create nodes
    # can't use the value in the parameter until we return the launch description.
    # thus, the lists, allow us to say "substitute `compass_type` with the actual value at runtime"
    compass_node = Node(package="compass", executable="compass", parameters=[config])
    gps_node = Node(package="gps", executable="gps", parameters=[config])

    camera_localizer_node = Node(
        package="obstacle_detection",
        executable="camera_localizer_node",
        name="camera_localizer",
        parameters=[config]
    )

    # create launch description for the luxonis depthai ros driver
    # camera_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         get_package_share_directory("depthai_ros_driver")
    #         + "/launch/camera.launch.py"
    #     ),
    # )

    return LaunchDescription([
        gps_node, 
        compass_node, 
        # camera_launch,
        camera_localizer_node
    ])