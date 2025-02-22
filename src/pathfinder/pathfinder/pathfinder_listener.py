import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import OccupancyGrid
from urc_intelsys_2024_msgs.msg import CART
from constants import MAP_TOPIC, GOAL_TOPIC, CARTESIAN_TOPIC, QOS


class PathfinderListener(Node):
    def __init__(self):
        super().__init__("pathfinder_listener")
        self.create_subscription(
            OccupancyGrid, MAP_TOPIC, lambda heading: print(heading), QOS
        )
        self.create_subscription(
            Float64, GOAL_TOPIC, lambda heading: print(heading), QOS
        )
        self.create_subscription(
            CART, CARTESIAN_TOPIC, lambda heading: print(heading), QOS
        )


def main():
    rclpy.init()
    try:
        rclpy.spin(PathfinderListener())
    except KeyboardInterrupt:
        print("Shutting down")
