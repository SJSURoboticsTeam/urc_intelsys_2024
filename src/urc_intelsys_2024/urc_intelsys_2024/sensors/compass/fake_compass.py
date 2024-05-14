from urc_intelsys_2024.sensors.compass.compass import Compass
import random
import rclpy


class FakeCompass(Compass):
    def __init__(self):
        super().__init__()

    def get_cur_angle(self) -> float:
        return random.random() * 360


def main():
    rclpy.init()
    rclpy.spin(FakeCompass())
    rclpy.shutdown()
