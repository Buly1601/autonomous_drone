import cv2
import numpy as np
from cv_bridge import CvBridge

class Camera:

    def __init__(self, dummy=False):
        # read image
        self.img = cv2.imread("./images/squares_2.png")
        #self.img = cv2.VideoCapture(0)
        #self.ret, self.frame = self.img.read()
        # TODO read from video real-time
        # convert image to hsv
        #self.hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
        self.hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
        # create threshold image
        self.lower = (0, 240, 160)
        self.upper = (30, 255, 255)
        self.threshold = cv2.inRange(self.hsv, self.lower, self.upper)
        # opencv ros bridge
        self.bridge = CvBridge()
        # subscribe to camera topic

        # call functions 
        self.morphology()
        self.draw_contours()
        self.get_square_center()
        self.get_center_img()
        if dummy:
            self.show_results()
        else:
            self.show_final()


    def image_callback(self, msg):
        """
        ROS image callback
        """
        self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow("Drone camera", self.img)
        cv2.waitKey(1) 


    def get_square_center(self):
        """
        Locates and draws the center of the square in the frame
        """
        # loop over the contours
        for c in self.contours:
            # compute the center of the contour
            M = cv2.moments(c)
            self.cX = int(M["m10"] / M["m00"])
            self.cY = int(M["m01"] / M["m00"])
            # draw the contour and center of the shape on the image
            cv2.circle(self.final, (self.cX, self.cY), 2, (0, 255, 0), -1)


    def get_center_img(self, show=False):
        """
        Locates and draws the center of the image with an x
        """
        # get width and height
        height = self.img.shape[0]
        width = self.img.shape[1]
        # get center
        self.center = (width//2, height//2)

        if show:
            print(self.center)
        
        # draw center
        self.img = cv2.circle(self.final, self.center, 2, (255, 0, 0), 2)
        # draw error circle
        self.img = cv2.circle(self.final, self.center, 20, (255, 0, 0), 1)


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
        self.contours = cv2.findContours(self.clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        self.contours = self.contours[0] if len(self.contours) == 2 else self.contours[1]

        self.final_unclean = self.img.copy()
        self.final = self.img.copy()
        for c in self.contours:
            cv2.drawContours(self.final_unclean, [c], 0,( 0,0,0), 2)
            # get rotated rectangle from contour
            rot_rect = cv2.minAreaRect(c)
            box = cv2.boxPoints(rot_rect)
            box = np.int0(box)
            # draw rotated rectangle on copy of img
            cv2.drawContours(self.final, [box], 0, (0,0,0), 2)



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




if __name__ =="__main__":
    camera = Camera()
