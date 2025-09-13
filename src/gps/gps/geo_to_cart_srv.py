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
                ("ref_latitude", 90.0),  # degrees
                ("ref_longitude", 90.0),  # degrees
            ],
        )
        # WGS-84
        self.A = 6378137.0  # semi-major axis [m]
        self.EE = 6.69437999013e-3  # first eccentricity squared

        # Reference angles (radians)
        self.phi0 = self.deg_to_rad(self.get_parameter("ref_latitude").value)
        self.lam0 = self.deg_to_rad(self.get_parameter("ref_longitude").value)
        self.get_logger().info(
            "Reference angles: %f, %f"
            % (180.0 / math.pi * self.phi0, 180.0 / math.pi * self.lam0)
        )

        # Precompute reference ECEF (assuming h = 0)
        self.x_ref, self.y_ref, self.z_ref = self.gnss_to_ecef(self.phi0, self.lam0)

        self.srv = self.create_service(
            GeoToCart, "geo_to_cart", self.geo_to_cart_callback
        )
        self.srv  # prevent unused

    def deg_to_rad(self, deg: float) -> float:
        return deg * math.pi / 180.0

    def gnss_to_ecef(self, lat_rad: float, lon_rad: float, h: float = 0.0):
        """Geodetic (φ, λ, h) -> ECEF (x, y, z)"""
        sinφ = math.sin(lat_rad)
        cosφ = math.cos(lat_rad)
        sinλ = math.sin(lon_rad)
        cosλ = math.cos(lon_rad)

        N = self.A / math.sqrt(1.0 - self.EE * sinφ * sinφ)

        x = (N + h) * cosφ * cosλ
        y = (N + h) * cosφ * sinλ
        z = (N * (1.0 - self.EE) + h) * sinφ
        return x, y, z

    def ecef_to_enu(self, x: float, y: float, z: float):
        """ECEF deltas -> ENU (relative to reference φ0, λ0)"""
        dx = x - self.x_ref
        dy = y - self.y_ref
        dz = z - self.z_ref

        sinφ0 = math.sin(self.phi0)
        cosφ0 = math.cos(self.phi0)
        sinλ0 = math.sin(self.lam0)
        cosλ0 = math.cos(self.lam0)

        # Standard ENU rotation
        e = -sinλ0 * dx + cosλ0 * dy
        n = -sinφ0 * cosλ0 * dx - sinφ0 * sinλ0 * dy + cosφ0 * dz
        # u (up) not returned but shown for completeness:
        # u =  cosφ0 * cosλ0 * dx + cosφ0 * sinλ0 * dy + sinφ0 * dz

        return e, n

    def gnss_to_cart(self, lat_deg: float, lon_deg: float):
        """Geodetic degrees -> local tangent plane EN (meters) relative to reference."""
        lat_rad = self.deg_to_rad(lat_deg)
        lon_rad = self.deg_to_rad(lon_deg)
        x, y, z = self.gnss_to_ecef(lat_rad, lon_rad)
        e, n = self.ecef_to_enu(x, y, z)
        return float(e), float(n)

    def geo_to_cart_callback(self, request, response):
        # request.inp.latitude / longitude assumed in degrees
        e, n = self.gnss_to_cart(request.inp.latitude, request.inp.longitude)
        response.out.x = int(e)
        response.out.y = int(n)
        return response


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(GeoToCartHandler())
    except KeyboardInterrupt:
        print("shutting down")
