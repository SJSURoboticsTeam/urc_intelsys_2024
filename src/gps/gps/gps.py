from abc import ABCMeta, abstractmethod
from typing import Tuple, Union
import math
from rclpy.node import Node
from urc_intelsys_2024_msgs.msg import GPS
from constants import GPS_TOPIC, QOS


class Util:
    """
    Methods were taken from AutoHelp
    """

    def get_distance(current_GPS: GPS, target_GPS: GPS) -> Union[GPS, None]:
        """
        Returns a tuple containing the distance between current_GPS and target_GPS
        The tuple is of the following format: (distanceInKM, distanceInMiles)

        """
        R_KM = 6373.0
        R_MI = 3958.8
        try:
            current_lat = math.radians(current_GPS.latitude)
            current_lon = math.radians(current_GPS.longitude)
            target_lat = math.radians(target_GPS.latitude)
            target_lon = math.radians(target_GPS.longitude)

            dis_lon = target_lon - current_lon
            dis_lat = target_lat - current_lat

            form1 = (
                math.sin(dis_lat / 2) ** 2
                + math.cos(current_lat)
                * math.cos(target_lat)
                * math.sin(dis_lon / 2) ** 2
            )
            form2 = 2 * math.atan2(math.sqrt(form1), math.sqrt(1 - form1))

            distanceKM = R_KM * form2
            distanceMi = R_MI * form2
            return distanceKM, distanceMi
        except:
            print("No GPS Data")

    def get_bearing(current_GPS: GPS, target_GPS: GPS) -> float:
        """Returns the angle between two GPS coordinates, relative to north

        PARAMS:
            current_GPS (tuple): (latitude, longitude)
            target_GPS (tuple):  (latitude, longitude)
        RETURNS:
            float. angle between the two coordinates
        """
        try:
            current_latitude = math.radians(current_GPS[1])
            current_longitude = math.radians(current_GPS[0])
            target_latitude = math.radians(target_GPS[1])
            target_longitude = math.radians(target_GPS[0])

            deltalog = target_longitude - current_longitude

            x = math.cos(target_latitude) * math.sin(deltalog)
            y = (math.cos(current_latitude) * math.sin(target_latitude)) - (
                math.sin(current_latitude)
                * math.cos(target_latitude)
                * math.cos(deltalog)
            )

            bearing = (math.atan2(x, y)) * (180 / 3.14)
            return bearing
        except:
            print("No GPS Data")

    def geographic_coordinates_to_relative_coordinates(
        cur_angle: float,
        cur_gps: GPS,
        target_gps: GPS,
    ) -> Tuple[float, float]:
        """
        Convert the target latitude/longitude into polar form of (theta, r)
        where theta is in radians
        """
        distance = Util.get_distance(cur_gps, target_gps)
        distance = distance[0] * 1000  # convert from km to m
        target_angle = Util.get_bearing(cur_gps, target_gps)

        # this is how much we need to turn
        relative_angle = cur_angle - target_angle
        # add 90* because the front of the rover is at rahul's 90* mark
        final_angle = relative_angle + 90
        # make positive angle
        if final_angle < 0:
            final_angle += 360

        # convert to radians before returning
        return (final_angle * math.pi / 180, distance)


class _GPS(Node, metaclass=ABCMeta):
    def __init__(self, gps_publish_seconds: float = 1.0) -> None:
        super().__init__("GPS")
        self.gps_publisher = self.create_publisher(GPS, GPS_TOPIC, QOS)

        self.gps_timer = self.create_timer(
            gps_publish_seconds,
            lambda: self.gps_publisher.publish(self.get_cur_gps()),
        )

    @abstractmethod
    def get_cur_gps(self) -> GPS:
        """
        Returns the current GPS position
        """
        pass
