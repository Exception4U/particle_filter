#!/usr/bin/env python

# Author : Akshay Raj
# Date :13/06/2106
# Extract features (SIFT,Harris,etc) from images and match them to corresponding image



import cv2
import math
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server as DynServer
import signal
import rosbag
import sys
#import robot.cfg.featuresConfig as Config
import reader


class Feature:

    def __init__(self):

        self.rd = reader.Reader()
        self.bridge=CvBridge()
        self.register()
        self.left_features = Image()
        self.process()




    def process(self):
        while(True):
            left_img = self.rd.getLeftImage()
            self.sift(left_img)

    def userQuit(self,signal,frame):
        rospy.loginfo("Reader is shutting down")
        sys.exit()


    def sift(self,img):
        # sift_left = cv2.xfeatures2d.SIFT_create()
        # kp = sift_left.detect(left_img,None)
        # self.left_features=cv2.drawKeypoints(left_img,kp)

        detector = cv2.FeatureDetector_create("SIFT")
        descriptor = cv2.DescriptorExtractor_create("SIFT")

        skp = detector.detect(img)
        skp, sd = descriptor.compute(img, skp)

        tkp = detector.detect(template)
        tkp, td = descriptor.compute(template, tkp)

        try:
            self.image_filter_pub.publish(self.bridge.cv2_to_imgmsg(self.left_features, encoding="bgr8"))


    	except CvBridgeError as e:
                rospy.logerr(e)

    def register(self):
        self.left_features_pub=rospy.Publisher("/features/left",Image)



if __name__=="__main__":
    rospy.init_node("features")
    feature=Feature()
    rospy.spin()
