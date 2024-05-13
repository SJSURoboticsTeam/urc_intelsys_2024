import urc_intelsys_2024.sensors.compass.LSM303 as LSM303
from urc_intelsys_2024.sensors.compass.compass import Compass


class ActualCompass(Compass):
    def __init__(self):
        super().__init__()

        self.lsm303 = LSM303.Compass()

    def get_cur_angle(self) -> float:
        return self.lsm303.get_heading()
