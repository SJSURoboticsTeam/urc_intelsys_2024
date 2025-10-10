import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from constants import IMAGE_TOPIC, QOS
from geometry_msgs.msg import PoseStamped
import cv2
from cv_bridge import CvBridge
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory


class KeyboardAlignment(Node):
    def __init__(self):
        super().__init__("keyboard_alignment")
        
        #subscribes to IMAGE_TOPIC
        self.create_subscription(Image, IMAGE_TOPIC, self.image_callback, QOS)
        #publishes PoseStamped message??
        self.publisher_ = self.create_publisher(PoseStamped, "keyboard_alignment_topic", QOS)

        #allows for you to have access to templatte image
        package_share_dir = get_package_share_directory("autonomous_typing")
        template_image_path = os.path.join(package_share_dir, "resources", "template_image.jpg")
        
        #load template image
        self.bridge = CvBridge()
        self.template = cv2.imread(template_image_path, cv2.IMREAD_GRAYSCALE)
        if self.template is None:
            self.get_logger().error(f"Failed to load template image: {template_image_path}")
            exit(1)

        self.get_logger().infor("KeyboardAlignmentNode running")

    def image_callback(self, msg):

        sift = self.sift_detector(msg.data)
        #corner = self.corner_detection(msg.data)

        self.publisher_.publish(sift)

        self.get_logger().info(f'Sift output: "{sift}"')

    def corner_detection(self):
        return
    
    def sift_detector(self):
        return