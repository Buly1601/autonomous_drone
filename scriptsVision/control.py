import camera
#import rospy
import sys
import time

class Drone:

    def __init__(self):
        self.camera = camera.Camera()
        self.align_to_square()
    

    def search(self, x):
        """
        Moves from current position -x to x
        """
        

    def manual_takeover(self):
        """
        When needed, after hitting the letter "m"
        it will give teleop commands priority
        """
    

    def align_to_square(self):
        """
        Aligns center of image to center of square with a 5-pixel 
        error threshold.
        """
        # align x
        while abs(self.camera.center[0] - self.camera.cX) > 20:
            if self.camera.center[0] - self.camera.cX > 20:
                # move x in left
                print("move left")
            elif self.camera.center[0] - self.camera.cX < 20:
                print("move right")
            else:
                print("center")    
            # !change to rospy.sleep()
            time.sleep(1)
        
            
        

if __name__ == "__main__":
    Drone()