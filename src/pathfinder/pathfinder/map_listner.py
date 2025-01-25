import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from constants import MAP_TOPIC, QOS


class MapListener(Node):
    def __init__(self):
        super().__init__("map_listener")
        path_finder=self.create_subscription(
            OccupancyGrid, MAP_TOPIC, lambda heading: print(heading), QOS
        )


def main():
    rclpy.init()
    try:
        rclpy.spin(MapListener())
    except KeyboardInterrupt:
        print("Shutting down")
