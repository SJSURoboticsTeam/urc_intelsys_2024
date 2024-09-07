from gps.gps import _GPS
from urc_intelsys_2024_msgs.msg import GPS
import rclpy
from std_msgs.msg import Float64


class FakeGPSCompass(_GPS):
    def __init__(self) -> None:
        super().__init__()

    def get_cur_gps(self) -> GPS:
        return GPS(
            longitude=Float64(data=float(100)), latitude=Float64(data=float(100))
        )


def main(args=None):
    rclpy.init(args=args)

    fake_gps = FakeGPSCompass()

    rclpy.spin(fake_gps)  # starts the node, blocking

    rclpy.shutdown()
