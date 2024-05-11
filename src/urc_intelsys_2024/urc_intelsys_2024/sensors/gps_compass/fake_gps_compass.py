from urc_intelsys_2024.sensors.gps_compass.gps_compass_class import _GPSCompass
from urc_intelsys_2024_msgs.msg import GPS
import rclpy
from urc_intelsys_2024.util.msg_creators import create_gps_msg


class FakeGPSCompass(_GPSCompass):
    def __init__(self) -> None:
        super().__init__()

    def get_cur_angle(self) -> float:
        return 45.0

    def get_cur_gps(self) -> GPS:
        return create_gps_msg(100, 100)


def main(args=None):
    rclpy.init(args=args)

    fake_gpsc = FakeGPSCompass()

    rclpy.spin(fake_gpsc)  # starts the node, blocking

    rclpy.shutdown()
