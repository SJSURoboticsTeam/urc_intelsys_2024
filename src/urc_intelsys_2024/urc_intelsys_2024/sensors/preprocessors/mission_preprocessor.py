from rclpy.node import Node
from std_msgs.msg import Float64
from urc_intelsys_2024_msgs.msg import GPS
from urc_intelsys_2024.constants import GPS_TOPIC, QOS, COMPASS_TOPIC


class MissionPreprocessor(Node):

    def __init__(self):
        super().__init__("mission_preprocessor")
        # needs to take in
        # gps, orientation, heading, obstacles, and aruco tags

        self.create_subscription(GPS, GPS_TOPIC, self._gps_handler, QOS)
        self.create_subscription(Float64, COMPASS_TOPIC, self._compass_handler, QOS)

    def _gps_handler(self, gps: GPS):
        # TODO - make this useful
        pass

    def _compass_handler(self, compass: Float64):
        # TODO - make this useful
        pass
