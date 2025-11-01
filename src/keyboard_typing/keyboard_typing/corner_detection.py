from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory
import matplotlib.pyplot as plt


class CornerDetection():
    def __init__(self, full_keyboard_template_path):
        #load template image
        self.corner_detection_template = cv2.imread(full_keyboard_template_path, cv2.IMREAD_COLOR)

        if self.corner_detection_template is None:
            print(f"Failed to load template image: {full_keyboard_template_path}")
            exit(1)

        print("CornerDetection running")

    def corner_detect(self):
        #harris detection
        gray = cv2.cvtColor(self.corner_detection_template, cv2.COLOR_BGR2GRAY)
        gray = np.float32(gray)
        dst = cv2.cornerHarris(gray,2,3,0.04)
        #result is dilated for marking the corners
        dst = cv2.dilate(dst,None)
        gray_uint8 = cv2.normalize(gray, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        vis_image = cv2.cvtColor(gray_uint8, cv2.COLOR_GRAY2BGR)
        # Threshold to detect strong corners
        vis_image[dst > 0.01 * dst.max()] = [0, 0, 255]  # Mark in red
        cv2.imwrite("corner_detection_result.jpg", vis_image)
        
        #Shi-Tomasi method --> bad at the moment
        #gray = cv2.cvtColor(self.corner_detection_template, cv2.COLOR_BGR2GRAY)
        #corners = cv2.goodFeaturesToTrack(gray, 25, 0.01, 10)
        #corners = np.int0(corners)

        #gray_uint8 = cv2.normalize(gray, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        #vis_image = cv2.cvtColor(gray_uint8, cv2.COLOR_GRAY2BGR)
        #for i in corners:
        #    x, y = i.ravel()
        #    cv2.circle(vis_image, (x, y), 10, (0, 0, 255), -1)

        #cv2.imwrite("corner_detection_result.jpg", vis_image)


        return
    