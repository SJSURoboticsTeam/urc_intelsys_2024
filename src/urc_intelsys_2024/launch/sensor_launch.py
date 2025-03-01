from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():  # all launch files need a function with this name
    # get parameters
    config = get_package_share_directory("urc_intelsys_2024") + "/config/config.yaml"

    # create nodes
    # can't use the value in the parameter until we return the launch description.
    # thus, the lists, allow us to say "substitute `compass_type` with the actual value at runtime"
    # compass
    compass_node = Node(package="compass", executable="compass", parameters=[config])
    quaternion_compass_node = Node(
        package="compass", executable="quaternion_publisher", parameters=[config]
    )
    # gps
    gps_node = Node(package="gps", executable="gps", parameters=[config])
    geo_to_cart_node = Node(
        package="gps", executable="geo_to_cart", parameters=[config]
    )
    geo_to_cart_service = Node(
        package="gps", executable="geo_to_cart_srv", parameters=[config]
    )
    gps_distance_service = Node(
        package="gps", executable="gps_distance_srv", parameters=[config]
    )

    # map/world frame related
    map_node = Node(package="map", executable="map", parameters=[config])
    world_frame_node = Node(package="map", executable="worldframe", parameters=[config])

    # task related
    task_manager_node = Node(
        package="control", executable="task_manager", parameters=[config]
    )

    path_finder_node = Node(
        package="pathfinder", executable="path_publisher", parameters=[config]
    )
    # create launch description for the luxonis depthai ros driver
    # camera_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         get_package_share_directory("depthai_ros_driver")
    #         + "/launch/camera.launch.py"
    #     ),
    # )
    camera_localizer_node = Node(
        package="obstacle_detection",
        executable="camera_localizer_node",
        name="camera_localizer",
        parameters=[config],
    )

    return LaunchDescription(
        [
            # gps package nodes
            gps_node,
            geo_to_cart_node,
            geo_to_cart_service,
            gps_distance_service,
            # compass package nodes
            quaternion_compass_node,
            compass_node,
            # map package nodes
            map_node,
            world_frame_node,
            # control package nodes
            task_manager_node,
            # camera_launch,
            path_finder_node,
            camera_localizer_node,
        ]
    )
