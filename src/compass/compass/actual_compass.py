import compass.LSM303 as LSM303
from compass.compass_type import Compass_


class ActualCompass(Compass_):
    def __init__(self):
        super().__init__()

        self.lsm303 = LSM303.Compass()

    def get_cur_angle(self) -> float:
        return self.lsm303.get_heading()
