#!/usr/bin/env python
import roslib
import rospy
import cv_bridge

import cv2
import numpy as np
from cv_bridge import CvBridge

from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import Empty
from sensor_msgs.msg import Image

class Camera:

    def _init_(self, dummy=False):

        # read image
        self.img = cv2.imread('hola.png')
        # TODO read from video real-timesquares
        # convert image to hsv
        self.hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
        # create threshold image
        self.bridge = CvBridge()
        self.lower = (20, 100, 100)
        self.upper = (30, 255, 255)
        self.threshold = cv2.inRange(self.hsv, self.lower, self.upper)
        # call functions 
        self.morphology()
        self.draw_contours()
        if dummy:
            self.show_results()
        else:
            self.show_final()

    def image_callback(self,msg):
        
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow("Imagen del Tópico", image)
        cv2.waitKey(1)  
    

    def morphology(self):
        """
        Applies morphology.
        Math used:
         - opening
         - closing
        Kernel is a 9x9 & 15x15 pixel ellipse
        """
        # apply morphology
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9,9))
        # refine image
        self.clean = cv2.morphologyEx(self.threshold, cv2.MORPH_OPEN, self.kernel)
        # apply morphology
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15,15))
        # fill holes
        self.clean = cv2.morphologyEx(self.threshold, cv2.MORPH_CLOSE, self.kernel)


    def draw_contours(self):
        """
        Draw countours based on the cleaned morphological image.
        """
        # get external contours
        contours = cv2.findContours(self.clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = contours[0] if len(contours) == 2 else contours[1]

        self.final_unclean = self.img.copy()
        self.final = self.img.copy()
        for c in contours:
            cv2.drawContours(self.final_unclean, [c], 0,( 0,0,0), 2)
            # get rotated rectangle from contour
            rot_rect = cv2.minAreaRect(c)
            box = cv2.boxPoints(rot_rect)
            box = np.int0(box)
            # draw rotated rectangle on copy of img
            cv2.drawContours(self.final,[box],0,(0,0,0),2)


    def show_results(self):
        """
        Saves the result and displays the images that were filtered 
        along the way.
        """
        # save result
        cv2.imwrite("threshold.jpg", self.threshold)
        cv2.imwrite("clean.jpg", self.clean)
        cv2.imwrite("final_unclean.png", self.final_unclean)
        cv2.imwrite("final.png", self.final)

        # display result
        cv2.imshow("threshold", self.threshold)
        cv2.imshow("clean", self.clean)
        cv2.imshow("final_unclean", self.final_unclean)
        cv2.imshow("final", self.final)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


    def show_final(self):
        """
        Shows the final result segmented.
        """
        cv2.imshow("final", self.final)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    


if _name_ =="_main_":
    camera = Camera(dummy=True)