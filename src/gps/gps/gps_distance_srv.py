"""
This file should contains the definition of a Node that answers a
GPSDistance service request
"""

from urc_intelsys_2024_msgs.srv import GPSDistance
from urc_intelsys_2024_msgs.msg import GPS
import rclpy
from rclpy.node import Node
import math


class Util:
    def get_distance(current_GPS: GPS, target_GPS: GPS) -> float:
        """
        Returns the distance between current_GPS and target_GPS
        in kilometers
        """
        R_KM = 6373.0
        current_lat = math.radians(current_GPS.latitude)
        current_lon = math.radians(current_GPS.longitude)
        target_lat = math.radians(target_GPS.latitude)
        target_lon = math.radians(target_GPS.longitude)

        dis_lon = target_lon - current_lon
        dis_lat = target_lat - current_lat

        form1 = (
            math.sin(dis_lat / 2) ** 2
            + math.cos(current_lat) * math.cos(target_lat) * math.sin(dis_lon / 2) ** 2
        )
        form2 = 2 * math.atan2(math.sqrt(form1), math.sqrt(1 - form1))

        distanceKM = R_KM * form2
        return distanceKM

    def get_bearing(current_GPS: GPS, target_GPS: GPS):
        """
        Returns the angle between two GPS coordinates

        Args:
            current_GPS (GPS): the current GPS coordinate
            target_GPS (GPS):  the target GPS coordinate
        Returns:
            out (float): angle between the two coordinates, in degrees

        """
        current_latitude = math.radians(current_GPS.latitude)
        current_longitude = math.radians(current_GPS.longitude)
        target_latitude = math.radians(target_GPS.latitude)
        target_longitude = math.radians(target_GPS.longitude)

        deltalog = target_longitude - current_longitude

        x = math.cos(target_latitude) * math.sin(deltalog)
        y = (math.cos(current_latitude) * math.sin(target_latitude)) - (
            math.sin(current_latitude) * math.cos(target_latitude) * math.cos(deltalog)
        )

        bearing = math.degrees(math.atan2(x, y))
        return bearing


class GPSDistanceHandler(Node):
    def __init__(self):
        super().__init__("gps_distance_handler")

        self.srv = self.create_service(
            GPSDistance, "gps_distance", self.gps_distance_callback
        )
        self.srv  # prevent unused variable warning

    def gps_distance_callback(
        self, request: GPSDistance.Request, response: GPSDistance.Response
    ):
        response.distance_in_km = Util.get_distance(request.current, request.target)
        response.bearing_in_degrees = Util.get_bearing(request.current, request.target)
        return response


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(GPSDistanceHandler())
    except KeyboardInterrupt:
        print("shutting down")
