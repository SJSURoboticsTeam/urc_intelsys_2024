import rclpy
from rclpy.node import Node
from constants import GPS_TOPIC, CARTESIAN_TOPIC
from urc_intelsys_2024_msgs.msg import GPS, CART
import math


class CartesianPublisher(Node):
    def __init__(self):
        super().__init__("Cartesian_Publisher")
        self.declare_parameters(
            namespace="",
            parameters=[
                # "name", default_value
                ("ref_latitude", 90),
                ("ref_longitude", 90),
            ],
        )
        self.publisher_ = self.create_publisher(CART, CARTESIAN_TOPIC, 10)

        self.subscription = self.create_subscription(
            GPS, GPS_TOPIC, self.listener_callback, 10
        )
        self.subscription  # prevent unused variable warning
        # semi-major axis constant
        self.A = 6378137
        # eccentricity constant
        self.EE = 6.69437999013 * (10 ** (-3))

        lat_rad = self.deg_to_rad(self.get_parameter("ref_latitude").value)
        long_rad = self.deg_to_rad(self.get_parameter("ref_longitude").value)

        self.x_ref, self.y_ref, self.z_ref = self.gnss_to_ecef(lat_rad, long_rad)

    def deg_to_rad(self, deg):
        return deg * math.pi / 180

    def gnss_to_ecef(self, lat, long):
        # convert to gnss to ecef(cartesian coordinate with center of earth as origin)
        denom = math.sqrt(1 - self.EE * math.sin(lat) ** 2)
        x = (self.A / denom) * math.cos(lat) * math.cos(long)
        y = (self.A / denom) * math.cos(lat) * math.sin(long)
        z = (self.A * (1 - self.EE) / denom) * math.sin(lat)

        return (x, y, z)

    def ecef_to_enu(self, lat, long, x, y, z):
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

    def gnss_to_cart(self, lat, long):
        lat_rad = self.deg_to_rad(lat)
        long_rad = self.deg_to_rad(long)

        x, y, z = self.gnss_to_ecef(lat_rad, long_rad)
        e, n = self.ecef_to_enu(lat_rad, long_rad, x, y, z)
        x, y = self.scale_to_map(10000, 1000, e, n)

        return x, y

    def scale_to_map(self, physical_size, map_size, e, n):
        # find scaling factor
        s = (map_size - 1) / physical_size

        x = math.floor(n / 2 + e * s)
        y = math.floor(n / 2 + n * s)

        return (x, y)

    def listener_callback(self, msg):
        newMsg = CART()
        newMsg.x, newMsg.y = self.gnss_to_cart(msg.latitude, msg.longitude)

        self.publisher_.publish(newMsg)
        self.get_logger().info("Publishing x-coordinate: %s" % newMsg.x)
        self.get_logger().info("Publishing y-coordinate: %s" % newMsg.y)


def main(args=None):
    rclpy.init(args=args)

    try:
        my_node = CartesianPublisher()
        rclpy.spin(my_node)
        my_node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        print("shutting down")
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        my_node.destroy_node()
        rclpy.shutdown()
