import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from urc_intelsys_2024_msgs.msg import GPS
from urc_intelsys_2024.sensors.gps_compass.constants import *


class GPSListener(Node):
    def __init__(self):
        super().__init__("GPSListener")
        self.create_subscription(GPS, GPS_TOPIC, lambda gps: print(gps), 10)
        self.create_subscription(
            Float64, COMPASS_TOPIC, lambda heading: print(heading), 10
        )


def main():
    rclpy.init()
    rclpy.spin(GPSListener())
    rclpy.shutdown()
