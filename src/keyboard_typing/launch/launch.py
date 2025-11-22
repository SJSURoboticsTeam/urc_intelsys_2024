from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    # create nodes
    # create launch description for the luxonis depthai ros driver

    config = get_package_share_directory("urc_intelsys_2024") + "/config/config.yaml"


    camera_node = Node(
        package="depthai_ros_driver", #use OAK-D driver package
        executable="camera_node",
        name="camera_node",
        #output="screen",
    )

    autonomous_typing_node = Node(
        package="keyboard_typing",
        executable="autonomous_typing",
        name="autonomous_typing",
        #output="screen",
        parameters=[config],
    )

    return LaunchDescription(
        [
            camera_node,
            autonomous_typing_node,
        ]
    )
