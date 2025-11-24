from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory
from matplotlib import pyplot as plt



class TemplateMatching():
    def __init__(self, template_path, image_path=None, image=None):
        print("Hi")
        #load template image
        #use imread so that is loads in grayscale already. --> might need to change if corner detection?
        self.template = cv2.imread(template_path, cv2.IMREAD_GRAYSCALE)
        if image_path is not None:
            self.full_keyboard = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
            #self.full_keyboard_copy = self.full_keyboard.copy()
        elif image is not None:
            self.full_keyboard = image
        self.full_keyboard_copy = self.full_keyboard.copy()

        if self.template is None:
            print(f"Failed to load template image: {template_path}")
            exit(1)


        print("TemplateMatching running")

    def template_match(self):
        w, h = self.template.shape[::-1]


        #None of these are going to work unless template image is the same size of the portion of the big image
        method = getattr(cv2, 'TM_CCOEFF_NORMED') #okay
        #method = getattr(cv2, 'TM_SQDIFF_NORMED')
        #method = getattr(cv2, 'TM_SQDIFF')

 
        # Apply template Matching
        res = cv2.matchTemplate(self.full_keyboard_copy, self.template, method)
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
 
        # If the method is TM_SQDIFF or TM_SQDIFF_NORMED, take minimum
        if method in [cv2.TM_SQDIFF, cv2.TM_SQDIFF_NORMED]:
            top_left = min_loc
        else:
            top_left = max_loc
        bottom_right = (top_left[0] + w, top_left[1] + h)
 
        cv2.rectangle(self.full_keyboard_copy, top_left, bottom_right, 255, 2)
 
        plt.subplot(121),plt.imshow(res, cmap = 'gray')
        plt.title('Matching Result'), plt.xticks([]), plt.yticks([])
        plt.subplot(122),plt.imshow(self.full_keyboard_copy,cmap = 'gray')
        plt.title('Detected Point'), plt.xticks([]), plt.yticks([])
        plt.suptitle(method)
 
        #plt.show()
        plt.savefig('template_match_result.png')
        return
    