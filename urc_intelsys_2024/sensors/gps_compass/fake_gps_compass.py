from typing import Tuple
from urc_intelsys_2024.sensors.gps_compass.gps_compass_class import _GPSCompass
import math


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

    def start_service(self):
        pass

    def stop_service(self):
        pass


def main():
    from datetime import datetime
    import time

    gps = FakeGPSCompass()
    start_time = datetime.now()
    while (datetime.now() - start_time).total_seconds() < 10:
        print(gps.geographic_coordinates_to_relative_coordinates(0.0, 0.0))
        time.sleep(1)
