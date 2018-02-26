#!/usr/bin/env python

# Author : Akshay Raj
# Date :08/06/2106
# The Reader class uses the Rosbag python api to step through  images in a bag sequence. It provides
# utility funcions like get_left_image and get_right_imaege. The images in the bag file are assumed to
# be rectified images as output by stereo_image_proc

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

class Reader:



    def __init__(self):
    #    self.bridge = CvBridge();
    #    self.left_camera_topic = '/stereo/left/image_rect_color'
#        self.right_camera_topic = '/stereo/right/image_rect_color'

        self.left_camera_topic = '/camera/synced/left'
        self.right_camera_topic = '/camera/synced/right'

        self.left_image = Image()
        self.right_image = Image()
        self.bridge=CvBridge()
        signal.signal(signal.SIGINT,self.userQuit)

        self.register()
    #    bag = rosbag.Bag('/home/akshayarajdayal/viso_related/second.bag')
        # for topic, msg, t in bag.read_messages(topics=['/camera/left/image_raw','/camera/right/image_raw']):
        #     print topic
        # bag.close()

    #    bag.close()
    #    print 'closing'

    def userQuit(self,signal,frame):
        rospy.loginfo("Reader is shutting down")
        sys.exit()

    def getLeftImage():
        return self.left_image

    def getRightImage():
        return self.right_image

    def register(self):

        self.left_image_sub = rospy.Subscriber(self.left_camera_topic,Image,self.cameraCallbackLeft)
        rospy.loginfo("Subscribed to left camera")
        rospy.loginfo(self.left_camera_topic)

        self.right_image_sub = rospy.Subscriber(self.right_camera_topic,Image,self.cameraCallbackRight)
        rospy.loginfo("Subscribed to right camera")
        rospy.loginfo(self.right_camera_topic)

    def cameraCallbackLeft(self,ros_image):
        #rospy.loginfo("in cam")
        #cv_image=self.rosimg2cv(ros_image)
       # self.circles(cv_image)

        try:
            frame=self.bridge.imgmsg_to_cv2(ros_image,ros_image.encoding)
        except CvBridgeError as e:
            rospy.logerr(e)
            rospy.loginfo("CvBridge error")
        self.left_image = frame
        rospy.loginfo("Got left image: "+ str(ros_image.header.stamp))

    def cameraCallbackRight(self,ros_image):
        #rospy.loginfo("in cam")
        #cv_image=self.rosimg2cv(ros_image)
       # self.circles(cv_image)
        try:
            frame=self.bridge.imgmsg_to_cv2(ros_image,ros_image.encoding)
        except CvBridgeError as e:
            rospy.logerr(e)
            rospy.loginfo("CvBridge error")
        self.right_image = frame
        rospy.loginfo("Got right image: "+ str(ros_image.header.stamp))


if __name__=="__main__":
    rospy.init_node("reader")
    reader=Reader()
    rospy.spin()
