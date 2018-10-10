#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
python 3.7 ve Linux 64 için miniconda indir :
https://repo.continuum.io/miniconda/Miniconda3-latest-Linux-x86_64.sh
Sonra indirdigin .sh dosyasını çalıştır
yönergeleri izleyerek kur
Kendi environmentini olustur :
conda create -n denemecv2 python
sonra gereken seyleri kur:
pip install opencv-python
pip install goprocam
pip install goprohero
pip install gopro
sonra once vava.py sonra lal.py calistir
'''

import rospy
import sys
#sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import numpy as np
import cv2
from sensor_msgs.msg import Image,Imu
from cv_bridge import CvBridge, CvBridgeError
import tf
import math
import time
import os
import sys
import subprocess


bridge = CvBridge()
cap = cv2.VideoCapture("udp://127.0.0.1:10000") 
rospy.init_node("GoPro_streamer_node", anonymous=True)
rate = rospy.Rate(10)
downcam = rospy.Publisher("/gozcu/goz", Image, queue_size=1)

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    try:
        image_message = bridge.cv2_to_imgmsg(frame, "bgr8")
        
    except CvBridgeError as e:
        print(e)
    # Our operations on the frame come here
    #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Display the resulting frame
    downcam.publish(image_message)
    cv2.imshow('frame',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    #rospy.spin()
    #rate.sleep()

# When everything done, release the capture

cap.release()
cv2.destroyAllWindows()


