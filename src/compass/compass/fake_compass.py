from compass.compass_type import Compass_
import random


class FakeCompass(Compass_):
    def __init__(self):
        super().__init__()

    def get_cur_angle(self) -> float:
        return random.random() * 360
