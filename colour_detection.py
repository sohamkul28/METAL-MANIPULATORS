#!/usr/bin/env python3
#You need to name this node "color_detector"
#By Aditya Bhat

import cv2
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

class Color_Detector(): 
    def __init__(self):
        # Params
        self.mode="Not cleaning"
        self.critical_colour_fraction=0.05 
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(10)

        # Publishers
        self.pub = rospy.Publisher("cleaning_mode", String, queue_size=10)

        # Subscribers
        self.sub = rospy.Subscriber("camera/rgb/image_raw", Image, self.callback)

    def callback(self, img):
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(img, desired_encoding="bgr8")
        
        # Green mask
        mask_green= cv2.inRange(cv_image, (100,200,100), (135, 255, 135))
        green_fraction= np.sum(mask_green)/(255*np.prod(mask_green.shape))
        
        # Blue mask
        mask_blue = cv2.inRange(cv_image, (200,100,100), (255, 135, 135))
        blue_fraction= np.sum(mask_blue)/(255*np.prod(mask_blue.shape))

        # Colour needs to take up 5% of screen to switch modes. critical_colour_fraction can be changed to suit your needs.

        if(blue_fraction>self.critical_colour_fraction and blue_fraction>=green_fraction): 
            self.mode="Sweep mode"
        elif(green_fraction>self.critical_colour_fraction and green_fraction>blue_fraction):
            self.mode="Suction mode"
        else:
            self.mode="Not cleaning"
        rospy.loginfo("Mode: "+ self.mode + "\nBlue fraction of image:" +str(blue_fraction)+"\nGreen fraction of image:"+ str(green_fraction))

    def start(self):
        rospy.loginfo("Node started...")
        while not rospy.is_shutdown():
            self.pub.publish(self.mode)
            self.loop_rate.sleep()

if __name__ == '__main__':
    rospy.init_node("color_detector", anonymous=True)
    my_node = Color_Detector()
    my_node.start()


