import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
from constants import IMAGE_TOPIC, QOS
from geometry_msgs.msg import PoseStamped
from math import atan2

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

        #self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters()

        #camera stuff needed to determine position:
        self.camera_matrix = np.load('camera_matrix.npy')
        self.dist_coeffs = np.load('dist_coeffs.npy')

    #figure out which aruco dictionary to use --> do we know this beforehand?
    def detect_aruco_dictionary(self, gray_image):
        for d in POSSIBLE_DICTS:
            aruco_dict = aruco.getPredefinedDictionary(d)
            parameters = aruco.DetectorParameters()
            corners, ids, _ = aruco.detectMarkers(gray_image, aruco_dict, parameters=parameters)

            if ids is not None and len(ids) >0:
                print("Detected tags from dictionary")
                return corners, ids, aruco_dict
            
        print("No aruco tag dictionary detected")
        return None, None, None
                

    #get image, detect aruco tags, estimate pose, publish pose
    def image_callback (self, msg):
        #convert ROS2 Image to OpenCV image --> convert to grayscale
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = self.detect_aruco_dictionary(gray_image)

        if ids is None or len(ids) < 4: #do we need exactly 4, or can we do with 3?
            self.get_logger().info("No aruco tags detected.")
            return
        
        #estimating tag poses
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, 0.02, self.camera_matrix, self.dist_coeffs)

        tag_positions = {int(ids[i]): tvecs[i][0] for i in range(len(ids))}
        ''' 
        Will look something like this:
        Tag0: [x0, y0, z0] --> 3d coordinates of all tags from tvecs
        Tag1: [x1, y1, z1]
        Tag2: [x2, y2, z2]
        Tag3: [x3, y3, z3]
        '''

        required_tags = [0, 1, 2, 3]

        if all(t in tag_positions for t in required_tags):
            pts = np.array([tag_positions[t] for t in required_tags])
            self.compute_pose(pts)
    
    #computes pose
    def compute_pose(self, pts):
        #compute normal vector for plane orientation by taking cross product of 2 edge vectors between tags
        v1 = pts[1] - pts[0] #edge 1
        v2 = pts[3] - pts[0] #edge 2 of keyboard
        normal = np.cross(v1, v2)
        normal /= np.linalg.norm(normal) #unit vector perpendicular to keyboard

        #compute center of keyboard which is just the average of the 4 tags, so robot can center itself
        center = np.mean(pts, axis=0)

        #target position: how far we want to be away from keyboard
        offset_distance = 0.30
        target_position = center - normal * offset_distance

        #calculate yaw from normal (robot should face keyboard) --> does the rotation
        yaw = atan2(normal[0], normal[2])

        #convert to quaternion?

        #publish pose message
        pose = PoseStamped()

        self.publisher.publish(pose)
        self.get_logger().info(f"Published target pose: pose = {target_position}")

        
        #cv2.Mat.outputImage = gray_image.clone();
        #cv2.aruco.drawDetectedMarkers(outputImage, markerCorners, markerIds);

def main(args=None):
    rclpy.init(args=args)
    node = ArucoAlignment()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()