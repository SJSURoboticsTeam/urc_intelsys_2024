import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from constants import MAP_TOPIC, GOAL_TOPIC, QOS


class PathfinderListener(Node):
    def __init__(self):
        super().__init__("pathfinder_listener")
        self.create_subscription(
            Float64, MAP_TOPIC, lambda heading: print(heading), QOS,
            Float64, GOAL_TOPIC, lambda heading: print(heading), QOS
        )


def main():
    rclpy.init()
    try:
        rclpy.spin(PathfinderListener())
    except KeyboardInterrupt:
        print("Shutting down")