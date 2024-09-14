from compass.actual_compass import ActualCompass
from compass.fake_compass import FakeCompass
from std_msgs.msg import Float64
from constants import COMPASS_TOPIC, QOS
from rclpy.node import Node
import rclpy


class CompassRunner(Node):
    def __init__(self):
        super().__init__("compass")

        self.declare_parameters(
            namespace="",
            parameters=[
                # "name", default_value
                ("compass_publish_seconds", 0.5),
                ("compass_type", "actual"),
            ],
        )

        if self.get_parameter("compass_type").value == "actual":
            self.compass = ActualCompass()
        elif self.get_parameter("compass_type").value == "fake":
            self.compass = FakeCompass()
        else:
            raise ValueError("Unknown compass type")

        compass_publish_seconds = self.get_parameter("compass_publish_seconds").value
        self.publisher = self.create_publisher(Float64, COMPASS_TOPIC, QOS)
        self.timer = self.create_timer(
            compass_publish_seconds,
            lambda: self.publisher.publish(Float64(data=self.compass.get_cur_angle())),
        )


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(CompassRunner())
    except KeyboardInterrupt:
        print("Shutting down")
