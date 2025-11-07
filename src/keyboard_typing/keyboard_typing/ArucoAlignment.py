import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
from constants import IMAGE_TOPIC, QOS
from geometry_msgs.msg import PoseStamped

POSSIBLE_DICTS = [
    aruco.DICT_4x4_50,
    aruco.DICT_5X5_100,
    aruco.DICT_6X6_250,
    aruco.DICT_7x7_1000
]

class ArucoAlignment(Node):
    def __init__(self):
        super().__init__('aruco_alignment')
        self.bridge = CvBridge()
        self.subscriber = self.create_subscription(Image, IMAGE_TOPIC, self.image_callback, QOS)
        self.publisher = self.create_publisher(PoseStamped, 'autonomous_typing', QOS)

        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters()

    def detect_aruco_dictionary(gray_image):
        for d in POSSIBLE_DICTS:
            aruco_dict = aruco.getPredefinedDictionary(d)
            parameters = aruco.DetectorParameters()
            corners, ids, _ = aruco.detectMarkers(gray_image, aruco_dict, parameters=parameters)

            if ids is not None and len(ids) >0:
                print("Detected tags from dictionary")
                return corners, ids, aruco_dict
            
        print("No aruco tag dictionary detected")
        return None, None, None
                

    def image_callback (self, msg):
        #convert ROS2 Image to OpenCV image --> convert to grayscale
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = self.detect_aruco_dictionary(gray_image)

        if ids is None:
            return
        
        #cv2.Mat.outputImage = gray_image.clone();
        #cv2.aruco.drawDetectedMarkers(outputImage, markerCorners, markerIds);