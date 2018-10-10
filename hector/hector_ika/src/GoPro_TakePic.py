#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image,Imu
from cv_bridge import CvBridge, CvBridgeError

cv_bridge = CvBridge()

rospy.init_node("GoPro_Picture_Taker", anonymous=True)
rate = rospy.Rate(10)


def GoPro_Callback(msg):
    gozcu_image_data = cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    print 'asdasdasd'
    cv2.imshow('gozcu_image_data',gozcu_image_data)
    cv2.waitKey(0)

downcam = rospy.Subscriber("/gopro/camera/picture", Image, GoPro_Callback)
    
rospy.spin()