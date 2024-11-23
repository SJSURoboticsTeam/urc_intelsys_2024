import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from constants import COMPASS_TOPIC, QOS



class CompassListener(Node):
    def __init__(self):
        super().__init__("compass_listener")
        self.create_subscription(
            Float64, COMPASS_TOPIC, lambda heading: print(heading), QOS
        )


def main():
    rclpy.init()
    try:
        rclpy.spin(CompassListener())
    except KeyboardInterrupt:
        print("Shutting down")
