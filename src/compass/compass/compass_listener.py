import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from constants import COMPASS_TOPIC, QOS


class CompassListener(Node):
    def __init__(self):
        super().__init__("CompassListener")
        self.create_subscription(
            Float64, COMPASS_TOPIC, lambda heading: print(heading), QOS
        )


def main():
    rclpy.init()
    rclpy.spin(CompassListener())
    rclpy.shutdown()
