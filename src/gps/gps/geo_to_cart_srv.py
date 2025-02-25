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

        self.ref_lat_rad = self.deg_to_rad(self.get_parameter("ref_latitude").value)
        self.ref_long_rad = self.deg_to_rad(self.get_parameter("ref_longitude").value)

        self.srv = self.create_service(
            GeoToCart, "geo_to_cart", self.geo_to_cart_callback
        )
        self.srv  # prevent unused

    def deg_to_rad(self, deg):
        return deg * math.pi / 180

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

        R = 6378137
        x = (
            2
            * R
            * math.asin(
                math.cos(self.ref_lat_rad)
                * math.sin((long_rad - self.ref_long_rad) / 2)
            )
        )
        y = R * (lat_rad - self.ref_lat_rad)

        return x, y

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
