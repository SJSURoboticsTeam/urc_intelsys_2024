from rclpy.node import Node
from std_msgs.msg import Float64
from abc import ABCMeta, abstractmethod
from urc_intelsys_2024.constants import COMPASS_TOPIC, QOS


class Compass(Node, metaclass=ABCMeta):
    def __init__(self, compass_publish_seconds: float = 0.1):
        super().__init__("Compass")
        self.publisher = self.create_publisher(Float64, COMPASS_TOPIC, QOS)
        self.timer = self.create_timer(
            compass_publish_seconds,
            lambda: self.publisher.publish(Float64(data=self.get_cur_angle())),
        )

    @abstractmethod
    def get_cur_angle(self) -> float:
        """
        Returns the current angle from North in degrees.

        @return - float - a float from [0, 360) representing the current clockwise
            angle from North.
        """
