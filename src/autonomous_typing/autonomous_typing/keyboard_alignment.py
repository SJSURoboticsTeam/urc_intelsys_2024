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
        #self.publisher_ = self.create_publisher(PoseStamped, "keyboard_alignment_topic", QOS)

        #allows for you to have access to template image
        package_share_dir = get_package_share_directory("autonomous_typing")
        TEMPLATE_IMAGE_PATH = os.path.join(package_share_dir, "resources", "template_image.jpg")
        
        #load template image
        self.bridge = CvBridge()
        #use imread so that is loads in grayscale already. --> might need to change if corner detection?
        self.template = cv2.imread(TEMPLATE_IMAGE_PATH, cv2.IMREAD_GRAYSCALE)

        if self.template is None:
            self.get_logger().error(f"Failed to load template image: {TEMPLATE_IMAGE_PATH}")
            exit(1)

        self.get_logger().info("KeyboardAlignmentNode running")

    def image_callback(self, msg):
        #convert ROS2 Image to OpenCV image --> convert to grayscale
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='brg8')
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        #call sift on the frame
        sift_output = self.sift_detector(gray_image)

        #corner = self.corner_detection(gray_image)

        #self.publisher_.publish(sift_output)
        #self.get_logger().info(f'Sift output: "{sift_output}"')

    def corner_detection(self):
        return
    
    def sift_detector(self, gray_frame):
        #initialize sift detector
        sift = cv2.SIFT_create()

        #TEMPLATE:
        #find keypoints of template image (grayed out version) --> can put a mask
        template_keypoints = sift.detect(self.template, None)
        self.get_logger.info(f"Number of Template Keypoints: {len(template_keypoints)}")
        #draw those keypoints --> use flags for better keypoints
        template_image_keypoints = cv2.drawKeypoints(self.template, template_keypoints, None)
        #make a new image with the keypoints on it
        cv2.imwrite('sift_keypoints_template_image.jpg', template_image_keypoints)

        #FRAME:
        #do the same
        frame_keypoints = sift.detect(gray_frame, None)
        self.get_logger.info(f"Number of Frame Keypoints: {len(frame_keypoints)}")
        frame_image_keypoints = cv2.drawKeypoints(gray_frame, frame_keypoints, None)
        cv2.imwrite('sift_frame_image_keypoints.jpg', frame_image_keypoints)
        
        return