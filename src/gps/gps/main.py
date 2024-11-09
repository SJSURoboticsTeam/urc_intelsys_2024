from rclpy.node import Node
from gps.actual_gps import ActualGPS
from gps.fake_gps import FakeGPS
from urc_intelsys_2024_msgs.msg import GPS
from constants import GPS_TOPIC, QOS
import rclpy


class GPSRunner(Node):
    def __init__(self):
        super().__init__("gps")

        self.declare_parameters(
            namespace="",
            parameters=[
                # "name", default_value
                ("gps_publish_seconds", 0.5),
                ("gps_type", "fake"),
            ],
        )

        if self.get_parameter("gps_type").value == "actual":
            self.gps = ActualGPS()
        elif self.get_parameter("gps_type").value == "fake":
            self.gps = FakeGPS()
        else:
            raise ValueError("Unknown GPS type")

        gps_publish_seconds = self.get_parameter("gps_publish_seconds").value
        self.gps_publisher = self.create_publisher(GPS, GPS_TOPIC, QOS)

        self.gps_timer = self.create_timer(
            gps_publish_seconds,
            lambda: self.gps_publisher.publish(self.gps.get_cur_gps()),
        )


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(GPSRunner())
    except KeyboardInterrupt:
        print("Shutting down")
