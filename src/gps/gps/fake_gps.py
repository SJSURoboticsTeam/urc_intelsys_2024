from gps.gps_type import GPS_
from urc_intelsys_2024_msgs.msg import GPS


class FakeGPS(GPS_):
    def __init__(self):
        super().__init__()
        self.i = 0

    def get_cur_gps(self) -> GPS:
        self.i += 1
        return GPS(
            longitude=float(100 + self.i / 20), latitude=float(100 + self.i / 20)
        )
