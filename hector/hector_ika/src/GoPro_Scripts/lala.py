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
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import numpy as np
import cv2
cap = cv2.VideoCapture("udp://127.0.0.1:10000")


while(True):
    cv2.waitKey(10)
    # Capture frame-by-frame
    ret, frame = cap.read()
    #cap.set(3, 858)
    #cap.set(4, 480)
    # Our operations on the frame come here
    #print (cap.get(3))
    # Display the resulting frame
    #cv2.namedWindow("frame", 0)
    #cv2.resizeWindow("frame", 1280, 720) 
    frame = cv2.resize(frame, (800, 600))
    cv2.imshow('frame',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
