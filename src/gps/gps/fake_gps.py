from gps.gps_type import GPS_
from urc_intelsys_2024_msgs.msg import GPS


class FakeGPS(GPS_):
    def get_cur_gps(self) -> GPS:
        return GPS(longitude=float(100), latitude=float(100))
