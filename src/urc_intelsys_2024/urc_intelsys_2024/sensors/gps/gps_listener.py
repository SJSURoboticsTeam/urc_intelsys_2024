import rclpy
from rclpy.node import Node
from urc_intelsys_2024_msgs.msg import GPS
from urc_intelsys_2024.constants import GPS_TOPIC, QOS


class GPSListener(Node):
    def __init__(self):
        super().__init__("GPSListener")
        self.create_subscription(GPS, GPS_TOPIC, lambda gps: print(gps), QOS)


def main():
    rclpy.init()
    rclpy.spin(GPSListener())
    rclpy.shutdown()
