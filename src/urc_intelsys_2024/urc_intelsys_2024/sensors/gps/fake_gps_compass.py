from urc_intelsys_2024.sensors.gps.gps import _GPS
from urc_intelsys_2024_msgs.msg import GPS
import rclpy
from urc_intelsys_2024.util.msg_creators import create_gps_msg


class FakeGPSCompass(_GPS):
    def __init__(self) -> None:
        super().__init__()

    def get_cur_gps(self) -> GPS:
        return create_gps_msg(100, 100)


def main(args=None):
    rclpy.init(args=args)

    fake_gps = FakeGPSCompass()

    rclpy.spin(fake_gps)  # starts the node, blocking

    rclpy.shutdown()
