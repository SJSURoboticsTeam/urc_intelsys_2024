from urc_intelsys_2024_msgs.msg import GPS
from std_msgs.msg import Float64


def create_gps_msg(latitude: float, longitude: float):
    return GPS(latitude=float(latitude), longitude=float(longitude))


def create_float64_msg(data: float):
    return Float64(data=float(data))
