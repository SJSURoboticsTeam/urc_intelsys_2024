from typing import Tuple
from urc_intelsys_2024.sensors.gps_compass.gps_compass_class import _GPSCompass
import math
import rclpy


class FakeGPSCompass(_GPSCompass):
    def __init__(self) -> None:
        super().__init__()

    def get_cur_angle(self) -> int:
        pass

    def get_cur_gps(self) -> Tuple[int, int]:
        pass

    def geographic_coordinates_to_relative_coordinates(
        self, target_latitude: float, target_longitude: float
    ):
        return (math.pi / 2, 4)  # arbitrary values


def main(args=None):
    rclpy.init(args=args)

    fake_gpsc = FakeGPSCompass()

    rclpy.spin(fake_gpsc)  # starts the node, blocking

    rclpy.shutdown()
