import rclpy
from rclpy.node import Node
from urc_intelsys_2024_msgs.msg import TDrive
from urc_intelsys_2024.constants import QOS


class Driver(Node):
    def __init__(self):
        super().__init__("driver")
        self.create_subscription(TDrive, "/drive", self.pass_msg, QOS)

    def pass_msg(self, msg: TDrive):
        print(msg)
        # TODO - put conversion to CAN frames here and send them to the CAN bus


def main():
    rclpy.init()
    driver = Driver()
    rclpy.spin(driver)
    rclpy.shutdown()
