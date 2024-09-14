import rclpy
from rclpy.node import Node
from urc_intelsys_2024_msgs.msg import GPS
from constants import GPS_TOPIC, QOS


class GPSListener(Node):
    def __init__(self):
        super().__init__("gps_listener")
        self.create_subscription(GPS, GPS_TOPIC, lambda gps: print(gps), QOS)


def main():
    rclpy.init()
    try:
        rclpy.spin(GPSListener())
    except KeyboardInterrupt:
        print("Shutting down")
