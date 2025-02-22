"""
This file should contain a node that answers a
GeoToCart service request

Ex: https://docs.ros.org/en/crystal/Tutorials/Writing-A-Simple-Py-Service-And-Client.html

"""

import rclpy
from rclpy.node import Node
from urc_intelsys_2024_msgs.srv import GeoToCart
import math


class GeoToCartHandler(Node):
    def __init__(self):
        super().__init__("geo_to_cart_handler")
        self.declare_parameters(
            namespace="",
            parameters=[
                # "name", default_value
                ("ref_latitude", 90),
                ("ref_longitude", 90),
            ],
        )
        # semi-major axis constant
        self.A = 6378137
        # eccentricity constant
        self.EE = 6.69437999013 * (10 ** (-3))

        lat_rad = self.deg_to_rad(self.get_parameter("ref_latitude").value)
        long_rad = self.deg_to_rad(self.get_parameter("ref_longitude").value)

        self.x_ref, self.y_ref, self.z_ref = self.gnss_to_ecef(lat_rad, long_rad)

        self.srv = self.create_service(
            GeoToCart, "geo_to_cart", self.geo_to_cart_callback
        )
        self.srv  # prevent unused

    def deg_to_rad(self, deg):
        return deg * math.pi / 180

    def gnss_to_ecef(self, lat, long):
        # convert to gnss to ecef(cartesian coordinate with center of earth as origin)
        denom = math.sqrt(1 - self.EE * math.sin(lat) ** 2)
        x = (self.A / denom) * math.cos(lat) * math.cos(long)
        y = (self.A / denom) * math.cos(lat) * math.sin(long)
        z = (self.A * (1 - self.EE) / denom) * math.sin(lat)

        return (x, y, z)

    def ecef_to_enu(self, lat, long, x, y, z) -> tuple[float, float]:
        # find distances
        # convert ecef to enu (apply rotation matrix)
        dx = x - self.x_ref
        dy = y - self.y_ref
        dz = z - self.z_ref

        e = -1 * math.sin(long) * dx + math.cos(long) * dy
        n = (
            -1 * math.sin(lat) * math.cos(lat) * dx
            - math.sin(lat) * math.sin(long) * dy
            + math.cos(lat) * dz
        )

        return (e, n)

    def gnss_to_cart(self, lat: float, long: float) -> tuple[float, float]:
        """
        Transforms a given (lat, long) pair, in radians,
        into a cartesian (x, y) pair (relative to the configured reference point)

        Args:
            lat (float): the latitude, in radians
            long (float): the longitude, in radians

        Returns:
            out (tuple[float, float]): a pair containing (x, y) in meters
        """
        lat_rad = self.deg_to_rad(lat)
        long_rad = self.deg_to_rad(long)

        x, y, z = self.gnss_to_ecef(lat_rad, long_rad)
        e, n = self.ecef_to_enu(lat_rad, long_rad, x, y, z)
        return int(e), int(n)

    def geo_to_cart_callback(self, request, response):
        """
        request has inp (GPS)
        response has out (CART)

        See urc_intelsys_2024_msgs/srv/GeoToCart.srv
        """
        print("in service geo_to_cart callback")
        response.out.x, response.out.y = self.gnss_to_cart(
            request.inp.latitude, request.inp.longitude
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(GeoToCartHandler())
    except KeyboardInterrupt:
        print("shutting down")
