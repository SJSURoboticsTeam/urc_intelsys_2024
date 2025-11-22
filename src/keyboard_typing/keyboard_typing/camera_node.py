import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from constants import IMAGE_TOPIC, QOS
from geometry_msgs.msg import PoseStamped
import cv2
from cv_bridge import CvBridge
import numpy as np
import os

class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")

        #subscriber
        self.create_subscription(Image, IMAGE_TOPIC, self.image_callback, QOS)
        #publisher
        self.publisher = self.create_publisher(PoseStamped, "camera_node", QOS)

        self.get_logger().info("CameraNode running")

    def image_callback(self, msg):
        #hi
        return None


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down camera node")

if __name__ == "__main__":
    main()
