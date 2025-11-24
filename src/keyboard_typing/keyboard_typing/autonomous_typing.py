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
from keyboard_typing.template_matching import TemplateMatching
from keyboard_typing.corner_detection import CornerDetection
from keyboard_typing.aruco_alignment import ArucoAlignment




class AutonomousTyping(Node):
    def __init__(self):
        super().__init__("autonomous_typing")

        #subscribes to IMAGE_TOPIC
        #self.create_subscription(Image, IMAGE_TOPIC, self.image_callback, QOS)
        self.create_subscription(Image, "/camera_node/depth/image_raw", self.depth_callback, 10)
        self.create_subscription(Image, "/camera_node/rgb/image_raw", self.rgb_callback, 10)
        #publishes PoseStamped message??
        self.publisher = self.create_publisher(PoseStamped, "autonomous_typing/pose", 10)

        self.bridge = CvBridge()

        #creates an instance of aruco alignment
        self.aruco_detector = ArucoAlignment(bridge=self.bridge)

        #allows for you to have access to template image
        package_share_dir = get_package_share_directory("keyboard_typing")
        self.FULL_KEYBOARD_IMAGE_PATH = os.path.join(package_share_dir, "resource", "full_keyboard_template_image.jpg")
        self.TEMPLATE_IMAGE_PATH = os.path.join(package_share_dir, "resource", "template_image.jpg")

        #load template image
        #use imread so that is loads in grayscale already. --> might need to change if corner detection?
        self.sift_template = cv2.imread(self.FULL_KEYBOARD_IMAGE_PATH, cv2.IMREAD_GRAYSCALE)

        if self.sift_template is None:
            self.get_logger().error(f"Failed to load template image: {self.FULL_KEYBOARD_IMAGE_PATH}")
            exit(1)

        self.get_logger().info("AutonomousTyping running")

    def rgb_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            #Aruco Detection
            pose = self.aruco_detector.get_keyboard_pose(msg)
            if pose is not None:
                self.publisher.publish(pose)
                self.get_logger().info(f"Published keyboard pose: {pose}")
            
            cv_image_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            template_matching_output = TemplateMatching(self.TEMPLATE_IMAGE_PATH, image=cv_image_gray)
            result = template_matching_output.template_match()
            self.get_logger().info(f"Template Matching Done")

            corner_detection_output = CornerDetection(image=cv_image)
            corner_detection_output.corner_detect()
            self.get_logger().info(f"Corner Detection Done")

            keypoints = self.sift_detector(cv_image)  # Pass current frame for keypoint detection


        except Exception as e:
            self.get_logger().error(f"Something broke in rgb_callback: {e}")
        #return None

    def depth_callback(self, msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        except Exception as e:
            self.get_logger().info(f"Something broke in depth_callback: {e}")
        return None



    def corner_detection(self):
        return
    
    def sift_detector(self, cv_image):
        #initialize sift detector
        sift = cv2.SIFT_create()

        gray_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        #TEMPLATE:
        #find keypoints of template image (grayed out version) --> can put a mask
        template_keypoints = sift.detect(self.sift_template, None)
        self.get_logger().info(f"Number of Template Keypoints: {len(template_keypoints)}")
        #draw those keypoints --> use flags for better keypoints
        template_image_keypoints = cv2.drawKeypoints(self.sift_template, template_keypoints, None)
        #make a new image with the keypoints on it
        cv2.imwrite('sift_keypoints_template_image.jpg', template_image_keypoints)

        #show image
        #cv2.imshow('SIFT Keypoints', template_image_keypoints)
        #cv2.waitKey(1)


        #FRAME:
        #do the same
        frame_keypoints = sift.detect(gray_frame, None)
        self.get_logger().info(f"Number of Frame Keypoints: {len(frame_keypoints)}")
        frame_image_keypoints = cv2.drawKeypoints(gray_frame, frame_keypoints, None)
        cv2.imwrite('sift_frame_image_keypoints.jpg', frame_image_keypoints)
        
        #show image
        #cv2.imshow('SIFT Keypoints', frame_image_keypoints)
        #cv2.waitKey(1)

        return template_image_keypoints#, frame_image_keypoints
    
def main(args=None):
    rclpy.init(args=args)
    node = AutonomousTyping()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down autonomous typing")


if __name__ == "__main__":
    main()
