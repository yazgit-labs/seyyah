#!/usr/bin/env python
# -*- coding: utf-8 -*-

#YAZGİT - Yapay Zeka ve Grüntü İşleme Topluluğu

#--     OpenCV Kütüphaneleri

import numpy as np
import cv2
import cv_bridge

#--     ROS Kütüphaneleri

import rospy                                                                    #Python için ros kütüphanesi
from sensor_msgs.msg import Image,Imu                                           #İKA ve İHAdan yön verisi alınmasını sağlar
from geometry_msgs.msg import Twist, PoseStamped,WrenchStamped,Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Int64
import actionlib, tf
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import euler_from_quaternion                             #imu sensöründen "quaternion" şeklinde alınan veriye "euler" formatına çevirmek için kullanılır
from geometry_msgs.msg import Pose2D

#--     Standart Python Kütüphaneleri

import tf
import math
import time
import os
import sys
import subprocess

#--     Drone İçin Kütüphaneler

from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil

#--     Genel Değişkenler
MIN_NOISE_CIRCUMFERENCE = 1.2 #Approximate value of robot's circumference
INFLATION_LAYER = 0.14
UAV_GOAL_HEIGHT = 6.3
PI = 3.41592654
UAV_MAX_LINEAR_VEL = 0.5
TAKEOFF_HEIGHT = 4.0
IKA_HEIGHT = 0.15 #Approximate value

#--     Görüntü İşleme Değişkenleri
WALL_INTERVAL = 0.5 #Distance between the walls
WALL_THICKNESS = 0.05
WALL_THRESHOLD = 80
CLEARING_THRESHOLD = 240
RED_MIN_HSV_LIMIT = np.array([0,1,0])
RED_MAX_HSV_LIMIT = np.array([54,255,255])
GREEN_MIN_HSV_LIMIT = np.array([1,0,0])
GREEN_MAX_HSV_LIMIT = np.array([255,255,255])

BALANCE = 0.0

#--     Kamera Değişkenleri

HFOV = 1.647590814 #Radyan cinsinde kameranın hfov(yataydaki görüş açısı) değeri
WIDTH_RATIO = 800 #Kamera görüntüsünün yataydaki piksel sayısı

#Kamera Kalibrasyon Değerleri
#Kameradaki balıkgözü (fisheye) etkisini silmek için kullanılan python kodunun verdiği değerler
#GoPro stream Kalibrasyonu

K=np.array([[141.4269447535774, 0.0, 159.94645338424988], [0.0, 141.62282991367255, 124.59672331705613], [0.0, 0.0, 1.0]])
D=np.array([[0.04355826188101876], [0.054267279425115604], [-0.06931847347137489], [0.03650075042359531]]) 


#GoPro fotoğraf kalibrasyonu

p_K=np.array([[1510.2287647885526, 0.0, 1270.052499454168], [0.0, 1496.7714262220325, 990.7537347787629], [0.0, 0.0, 1.0]])
p_D=np.array([[0.06141866561812304], [-0.10007724964446188], [0.46830128995437087], [-0.6774650919133068]])

UNDISTORTION_COEFFICIENT = 1.336150798 #Fotoğrafa düzeltme işlemi uygulandıktan sonra haritanın boyutundaki sapma

#UAV = İHA
#UGV = İKA

#Oryantasyon = Nereye baktığın (Quaternion ya da Euler formatında)(Roll, Pitch, Yaw)
#Pozisyon = Nerede olduğun (X, Y, Z)
#Pose = Oryantasyon + Pozisyon
#Odometre = Ne kadar yol gittiğini bulan sensörler. Pozisyon değişimini bulur
#Gps mutlak pozisyon bulan sensör. Bir altimetre'de kullanılırsa Z eksenide bulunur
#Kara araçları için Gps yerine motor encoder'ları iş görür (Patinaj tehlikeli)
#Accelerometer, magnetometer ve gyroscope ile mutlak oryantasyon bulunur
#Üstte ki üçünü birleştirmek için sensor fusion işlemi gerekli
#Üstte ki üçünden sadece ikisi aynı işi yapabilir ama üçü yanyana daha başarılı
#Yani üstte ki üçü (Roll, Pitch, Yaw)'a direk karşılık gelmez

class RobotManager:
    def __init__(self): 
        
        #--     Veri havuzu         
        
        self.downcam_image_data = None
        self.uav_odom = None
        self.rate = rospy.Rate(10)
        
        #-- İşaretler
        self.seyyahThetaFlag = 0                #Seyyahtan gelen ilk "theta" değerini kaydetmek için kullanılır. Böylelikle yeşil noktaya vardığından yönünü başlangıçtaki yönününe çevirir ikinci harita tam oturmuş olur                                           
        self.goHomeFlag = 0                     #İHA fotoğraf çektikten sonra değer '1' olarak değiştirilir, değer '1' olduğu anda iniş başlar.
        
        #--     Hız değişkenleri            
        
        self.vel_gozcu = Twist()                   #IHA'nın hız değeri tanımlanır
        self.vel_seyyah = Twist()                  #İKA'nın hız değeri tanımlanır
        
        #--     Düzeltme işlemi için parametreler           
        self.DIM=(320, 240)
        self.DIM2=None
        self.DIM3=None
        
        self.p_DIM=(2560, 1920)
        self.p_DIM2 = None
        self.p_DIM3 = None
        
        self.cv_bridge = cv_bridge.CvBridge()
        
        #--     Move base için değişkenler
        
        self.map_flag = 1                       #ikinci map'ı map_server'a göndermek için
        self.move_base_status = None
        self.movebase_callback_flag = 0
        self.status = None                      #Move base status
        
        #--     Actionlib Bölümü           
        
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
        #--     Subscribe bölümü            
        
        rospy.Subscriber("/gozcu/goz",Image,self.gozcu_callback)
        rospy.Subscriber("/gopro/camera/picture", Image, self.GoPro_Callback)
        rospy.Subscriber("/move_base/status",GoalStatusArray,self.move_base_status_callback)
        rospy.Subscriber("/seyyah/pose2d",Pose2D,self.seyyah_odom_callback)
        
        #--     Publish bölümü          
        
        self.seyyah_cmd_vel = rospy.Publisher("/seyyah/cmd_vel", Twist, queue_size=1)
        self.GoPro = rospy.Publisher("/gopro/camera/take_picture", Int64, queue_size=1)
        self.sharing_goal = rospy.Publisher("/Goal_Share",PoseStamped,queue_size = 0)

        #--     Setup the commanded flying speed
        
        self.gnd_speed = 1 # [m/s]

        #--     Global velocity logs
        
        self.velocityLogList = []
        self.vehicle = None

    #           callback bölümü         #
        
    def GoPro_Callback(self,msg):
        self.GoPro_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        
        print 'asdasdasasddddddddddddddddddddddddd'
        
    def gozcu_callback(self,msg):    
      try:
          #asd = 2
          self.gozcu_image_data = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
          while self.gozcu_image_data is None:
              self.rate.sleep()  
          self.img = self.gozcu_image_data
          self.DIM1 = self.img.shape[:2][::-1]  #dim1 is the dimension of input image to un-distort
          assert self.DIM1[0]/self.DIM1[1] == self.DIM[0]/self.DIM[1], "Image to undistort needs to have same aspect ratio as the ones used in calibration"
          if not self.DIM2:
            self.DIM2 = self.DIM1
            #print 'asdasd'
          if not self.DIM3:
            self.DIM3 = self.DIM1

          scaled_K = K * self.DIM1[0] / self.DIM[0]  # The values of K is to scale with image dimension.
          scaled_K[2][2] = 1.0  # Except that K[2][2] is always 1.0
          # This is how scaled_K, dim2 and balance are used to determine the final K used to un-distort image. OpenCV document failed to make this clear!
          new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(scaled_K, D, self.DIM2, np.eye(3), balance=BALANCE)
          map1, map2 = cv2.fisheye.initUndistortRectifyMap(scaled_K, D, np.eye(3), new_K, self.DIM3, cv2.CV_16SC2)
          self.undistorted_img = cv2.remap(self.img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
      except cv_bridge.CvBridgeError:
          return
  
      self.img_cam = self.undistorted_img
      #self.img_cam = cv2.resize(self.img_cam , (800, 600)) #This is for using with imshow

      self.img_cam = cv2.cvtColor(self.img_cam, cv2.COLOR_BGR2GRAY)
      ret, self.img_cam = cv2.threshold(self.img_cam, WALL_THRESHOLD, 255, cv2.THRESH_BINARY_INV)
      (rows,cols) = self.img_cam.shape

      cv2.line(self.img_cam,(0,rows/2),(cols,rows/2),(0,255,255),15)
      cv2.line(self.img_cam,(cols/2,0),(cols/2,rows),(0,255,255),15)

      roi_t = self.img_cam[0:rows/2,0:cols]
      roi_b = self.img_cam[rows/2:rows,0:cols]
      roi_l = self.img_cam[0:rows,0:cols/2]
      roi_r = self.img_cam[0:rows,cols/2:cols]

      _, roi_t_contours, hier = cv2.findContours(roi_t,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
      _, roi_b_contours, hier = cv2.findContours(roi_b,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
      _, roi_l_contours, hier = cv2.findContours(roi_l,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
      _, roi_r_contours, hier = cv2.findContours(roi_r,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)

      self.t = 0
      self.b = 0
      self.r = 0
      self.l = 0

      for i in roi_t_contours:
        self.t = self.t + cv2.contourArea(i)
      for i in roi_b_contours:
        self.b = self.b + cv2.contourArea(i)
      for i in roi_l_contours:
        self.l = self.l + cv2.contourArea(i)
      for i in roi_r_contours:
        self.r = self.r + cv2.contourArea(i)

      font = cv2.FONT_HERSHEY_SIMPLEX
      cv2.putText(self.img_cam,('Height : %.2f' %(self.getALT())),(5,20),font,0.5,(255,255,255),1,cv2.LINE_AA)
      cv2.putText(self.img_cam,('Vel x : %.2f' %(self.vel_gozcu.linear.x)),(5,40),font,0.5,(255,255,255),1,cv2.LINE_AA)
      cv2.putText(self.img_cam,('Vel y : %.2f' %(self.vel_gozcu.linear.y)),(5,60),font,0.5,(255,255,255),1,cv2.LINE_AA)
      cv2.putText(self.img_cam,('Vel z : %.2f' %(self.vel_gozcu.linear.z)),(5,80),font,0.5,(255,255,255),1,cv2.LINE_AA)
      cv2.putText(self.img_cam,('Vel theta : %.2f' %(self.vel_gozcu.angular.z)),(5,100),font,0.5,(255,255,255),1,cv2.LINE_AA)
      cv2.putText(self.img_cam,('Area Top : %.2f' %(self.t)),(cols/2-100,20),font,0.5,(255,255,255),1,cv2.LINE_AA)
      cv2.putText(self.img_cam,('Area Bottom : %.2f' %(self.b)),(cols/2-100,rows-10),font,0.5,(255,255,255),1,cv2.LINE_AA)
      cv2.putText(self.img_cam,('Area Left : %.2f' %(self.l)),(3,rows/2+5),font,0.5,(255,255,255),1,cv2.LINE_AA)
      cv2.putText(self.img_cam,('Area Right : %.2f' %(self.r)),(cols-180,rows/2+5),font,0.5,(255,255,255),1,cv2.LINE_AA)
      
    def goal_callback(self,msg):
        self.pose = msg
        self.movebase_callback_flag = 1
        print "Goal Coodinates : ",msg.x,msg.y,msg.z
        if(self.movebase_callback_flag == 1):
            self.move_base.wait_for_server()
            #self.publishgoal(self.pose)# Kontrol nodunda oluşturduğumuz "goal" değişkeninin değerleri move_base'e gönderilir

    def move_base_status_callback(self,msg):
        self.status = msg
        while self.status == None:
                self.rate.sleep()
            
        self.move_base_status = msg.status_list[0].text
        print self.move_base_status
        
    def seyyah_odom_callback(self,msg):
        if self.seyyahThetaFlag == 0:
            self.seyyahInitTheta = msg.theta                #Seyyahtan yayınlanan ilk yön verisi kaydedilir.
            self.seyyahThetaFlag = 1
        self.seyyahXPose = msg.x
        self.seyyahYPose = msg.y
        self.seyyahTheta = msg.theta       
        
    #           Drone Fonksiyonları         #
    
    def droneCode_setup(self):
        print('Connecting to vehicle')
        self.vehicle = connect('udpin:0.0.0.0:14550',wait_ready=True)
        ###REAL VECIHLE BLOCK
        
    def printALT(self):
        #Show altitude
        v_alt = self.vehicle.location.global_relative_frame.alt
        print(">> Altitude = %.1f m"%v_alt)

    def getALT(self):
        #print 'altitude',self.vehicle.location.global_relative_frame.alt
        return self.vehicle.location.global_relative_frame.alt

    def printATT(self):
        #Show attitude
        v_alt = self.vehicle.location.global_relative_frame
        print(v_alt)

    def logVEL(self,vx,vy,vz):
        velList = [vx,vy,vz]
        self.velocityLogList.append(velList)

    def specialRTL(self):
        print("Ozel geri donus fonksyinu devrede!!!")
        for i in reversed(self.velocityLogList):
            self.set_velocity_body(i[0], i[1], i[2])
            self.printATT()
            time.sleep(1)
        print("Inis modu devrede!!!")
        self.vehicle.mode = VehicleMode("LAND")
    
    #-- Define arm and takeoff
    def arm_and_takeoff(self,altitude):
        print("Arming motors")
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        while not self.vehicle.armed: time.sleep(1)

        print("Taking Off")
        self.vehicle.simple_takeoff(altitude)
        timeout = time.time() + 10
        while True:
            v_alt = self.vehicle.location.global_relative_frame.alt
            print(">> Altitude = %.1f m"%v_alt)
            if v_alt >= altitude - 1.0:
                print("Target altitude reached")
                break
            elif time.time() > timeout:
                print("Can't reached target altitude !")
                break

            time.sleep(1)
        
    #-- Define the function for sending mavlink velocity command in body frame
    def set_velocity_body(self, vx, vy, vz):
        """ Remember: vz is positive downward!!!
        http://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
        
        Bitmask to indicate which dimensions should be ignored by the vehicle 
        (a value of 0b0000000000000000 or 0b0000001000000000 indicates that 
        none of the setpoint dimensions should be ignored). Mapping: 
        bit 1: x,  bit 2: y,  bit 3: z, 
        bit 4: vx, bit 5: vy, bit 6: vz, 
        bit 7: ax, bit 8: ay, bit 9:
        
        
        """
        print("hiz verisi: %f , %f , %f"%(vx,vy,vz))
        self.logVEL(-vx,-vy,-vz)
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
                0,
                0, 0,
                mavutil.mavlink.MAV_FRAME_BODY_NED,
                0b0000111111000111, #-- BITMASK -> Consider only the velocities
                0, 0, 0,        #-- POSITION
                vx, vy, vz,     #-- VELOCITY
                0, 0, 0,        #-- ACCELERATIONS
                0, 0)
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()
            
    def gozcu_mover(self,const_lin = 0.1,const_ang = 0.52,angel_tolerance = 0.087266463):
        while self.gozcu_image_data is None:
            self.rate.sleep()
        self.rate.sleep()   
        #self.t görüntünün ortadan üst kısmı, self.b alt kısmı, self.r sağ ve self.l sol kısmındaki beyaz alanların büyüklüğünü temsil eder.
        #Bu alaları resmin dört tarafında da eşit olmasını sağlamaya çalışarak ihaya hız komutu verir.

        #Büyük alanı küçük alana bölerek aradaki kat farkına göre hız komutu yayınlanır. 0.0001 sabitinin sebebi küçük alanın (paydanın) sıfır olması durumunda programın durmasını engellemek.
        if(self.getALT()>3.5):
        
            if self.t > self.b:
                self.vel_gozcu.linear.y = -1.5*const_lin*(self.t - self.b)/(self.b + 0.0001)*2.0 #-- 

            if self.t == self.b:
                self.vel_gozcu.linear.y = 0.0

            if self.b >self.t:
                self.vel_gozcu.linear.y = -1.5*const_lin*(self.t - self.b)/(self.t + 0.0001)*2.0

            if self.r > self.l:
                self.vel_gozcu.linear.x = -1.5*const_lin*(self.l - self.r)/(self.l + 0.0001)*2.0

            if self.r == self.l:
                self.vel_gozcu.linear.x = 0.0
            
            if self.l > self.r:
                self.vel_gozcu.linear.x = -1.5*const_lin*(self.l - self.r)/(self.r + 0.0001)*2.0

            print 'self.vel_gozcu.linear.x, self.vel_gozcu.linear.y',self.vel_gozcu.linear.x, self.vel_gozcu.linear.y

            ##Eğer alanlar arasındaki fark devasa ise aşırı büyük hız komutları yayınlanacaktır.
            ##Bunu engellemek için eğer hız değeri belirli bir eşiğin üzerinde ise (0.5 sembolik) onu sabit bir sayıya sabitleyerek ihanın daha stabil davranması sağlanır
            if self.vel_gozcu.linear.x >= UAV_MAX_LINEAR_VEL:
                self.vel_gozcu.linear.x = UAV_MAX_LINEAR_VEL

            if self.vel_gozcu.linear.x < -UAV_MAX_LINEAR_VEL:
                self.vel_gozcu.linear.x = -UAV_MAX_LINEAR_VEL

            if self.vel_gozcu.linear.y >= UAV_MAX_LINEAR_VEL:
                self.vel_gozcu.linear.y = UAV_MAX_LINEAR_VEL

            if self.vel_gozcu.linear.y < -UAV_MAX_LINEAR_VEL:
                self.vel_gozcu.linear.y = -UAV_MAX_LINEAR_VEL
                


            self.set_velocity_body(self.vel_gozcu.linear.x, self.vel_gozcu.linear.y, 0)
        
        time.sleep(1)
    #           İKA Bölümü      #
    
    def seyyah_yaw_equalizer(self,const = 0.6,tolerance = 0.03):
        while abs(self.seyyahInitTheta - self.seyyahTheta) > tolerance:     
            self.vel_seyyah.angular.z = (self.seyyahInitTheta - self.seyyahTheta)*const
            self.seyyah_cmd_vel.publish(self.vel_seyyah)
            print 'Yaw Equaling'
            
        self.vel_seyyah.angular.z = 0    
        self.seyyah_cmd_vel.publish(self.vel_seyyah)  
        
    #           Görüntü İşleme Bölümü           #

    def is_there_red_and_green(self):#Yeşil ve Kırmızıyı aynı anda görüp görmediğini kontrul eder(hsv limit değerlerinin değiştirilmesi gerekebilir
        red_flag = False
        green_flag = False

        self.rate.sleep()

        img = self.gozcu_image_data

        hsv_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        img_red = hsv_image
        img_green = hsv_image

        img_red = cv2.inRange(img_red, RED_MIN_HSV_LIMIT, RED_MAX_HSV_LIMIT)
        img_green = cv2.inRange(img_green, GREEN_MIN_HSV_LIMIT, GREEN_MAX_HSV_LIMIT)

        _, red_contours, hier = cv2.findContours(img_red,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
        _, green_contours, hier = cv2.findContours(img_green,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)

        for cnt in red_contours:
            if cv2.contourArea(cnt)>10:
                red_flag = True
            if cv2.contourArea(cnt)<10:
                red_flag = False


        for cnt in green_contours:
            if cv2.contourArea(cnt)>10:
                green_flag = True
            if cv2.contourArea(cnt)<10:
                green_flag = False

        if red_flag is True and green_flag is True:
            return True #Kameramda hem yeşil hem de kırmızı renk görüyorum

        else:
            return False

    def image_croper(self,iha_height,photograph): #Bu fonksiyon sayesinde aldığımız görüntüyü güzelce kırpıp düzeltiyoruz.
        try:
            self.p_DIM1 = self.photograph.shape[:2][::-1]  #dim1 is the dimension of input image to un-distort
            assert self.p_DIM1[0]/self.p_DIM1[1] == self.p_DIM[0]/self.p_DIM[1], "Image to undistort needs to have same aspect ratio as the ones used in calibration"
            if not self.p_DIM2:
                self.p_DIM2 = self.p_DIM1
                #print 'asdasd'
            if not self.p_DIM3:
                self.p_DIM3 = self.p_DIM1

            scaled_K = p_K * self.p_DIM1[0] / self.p_DIM[0]  # The values of K is to scale with image dimension.
            scaled_K[2][2] = 1.0  # Except that K[2][2] is always 1.0
            # This is how scaled_K, dim2 and balance are used to determine the final K used to un-distort image. OpenCV document failed to make this clear!
            new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(scaled_K, p_D, self.p_DIM2, np.eye(3), balance=BALANCE)
            map1, map2 = cv2.fisheye.initUndistortRectifyMap(scaled_K, p_D, np.eye(3), new_K, self.p_DIM3, cv2.CV_16SC2)
            self.undistorted_photo = cv2.remap(photograph, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        except cv_bridge.CvBridgeError:
            return
        self.resolution = 2.0*iha_height*math.tan(HFOV/2.0)/(WIDTH_RATIO*UNDISTORTION_COEFFICIENT)
        self.undistorted_photo = cv2.resize(self.undistorted_photo, (800,600))
        img_maze = self.undistorted_photo
        h,w,_ = img_maze.shape
        self.iha_height = iha_height
        cv2.imwrite('./Orjinal_Duzeltilmis.png',img_maze)
        
        #kernel = np.ones((8, 8), np.uint8)#Çerçeveyi bir miktar genişleterek tüm labirentin alınmasını kolaylaştırır
        bluePoint = 0
        bluePointX = 0
        bluePointY = 0 
        self.height, self.width,_ = img_maze.shape
        for x in range(0,self.width):
            for y in range(0,self.height):
                px = img_maze[y, x]
                if 60<=int(px[0]) and (int(px[0]) - int(px[2])) >= 20 and (int(px[0]) - int(px[1])) >= 20:
                    img_maze[y,x] = 255,255,255
                    #print px
                    bluePoint+=1
                    bluePointX = bluePointX + x
                    bluePointY = bluePointY + y

        bluePointX = bluePointX / bluePoint
        bluePointY = bluePointY / bluePoint
        
        gray = cv2.cvtColor(img_maze, cv2.COLOR_BGR2GRAY)

        cv2.imwrite('./Gri.png',gray)

        ret, thresh = cv2.threshold(gray, WALL_THRESHOLD, 255, cv2.THRESH_BINARY_INV)
        
        cv2.imwrite('./Dijital.png',thresh)
        dilation_kernel_dim = int((WALL_INTERVAL + 0.1) / self.resolution) # Çerçeve bir miktar geniş olsun diye duvarları 0.3 metre kadar kalınlaştırıyoruz

        closing_kernel_dim = int((WALL_INTERVAL) / 2*self.resolution) # Close işlminde kullnılacak matrisin
                                            # satır ve sütün sayıları gerçekte 0.3 metreye tekabul
                                            # edecek şekilde ayarlanır
                                            # 0.3 metre de kalınlaştırma olduğu için labirent tek blok haline gelir ve çerçeve hatasız bir şekilde oluşturulur


        closing_kernel = np.ones((closing_kernel_dim , closing_kernel_dim ), np.uint8)

        dilation_kernel = np.ones((dilation_kernel_dim, dilation_kernel_dim), np.uint8)
        
        thresh_dilated = cv2.dilate(thresh, dilation_kernel, iterations=1) # Genişletme işlemi

        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, closing_kernel,iterations=1) # Kapatma işlemi
        
        cv2.imwrite('./Kapatma.png',thresh)
        
        #Bu iki işlem sonunda labirent binary image'e dönüştüğünde tek bir beyaz blok'a dönüşür ve çerçeve oluşturmak daha kolay olur


        thresh, contours,hierarchy = cv2.findContours(thresh_dilated,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

        length = []
        max_len = 0
        for i in range(len(contours)):
                length.insert(i,cv2.arcLength(contours[i],True))
                if length[i]>max_len:
                        max_len = length[i]

        for j in range(len(contours)):
                if length[j] == max_len:
                        break

        cnt = contours[j]

        #ROTATING THE IMAGE

        rect = cv2.minAreaRect(cnt)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        (mx,my),(mw,mh),angle = rect

        w,h,_ = img_maze.shape
        if angle>=-45:
            M = cv2.getRotationMatrix2D((mx,my),(angle),1)
        if angle<-45:
            M = cv2.getRotationMatrix2D((mx,my),(90+angle),1)

        thresh = cv2.warpAffine(img_maze,M,(h,w))

        cv2.imwrite('./Cevirme.png',thresh)
        ###########################3
        #Döndürme işi biter
        #Bu bölümde görüntüdeki kırmızı ve yeşil bölgelerin piksel koordinatları toplanıp piksel sayılarına bölünerek
        #kırmızı ve yeşil bölgenin orta noktalarının piksel koordinatları bulunur.
        #Ardından bu değerlerin görüntünün sol alt köşesi referans alınarak koordinat değerleri tekrar şekillendirilir
        #(map_server görüntünün sol alt köşesinin orijin kabul ettiği içi)
        #metre/piksel sabiti ile çarpılarak metre cinsinden koordinatları bulunur

        redPoint = 0
        redPointX = 0
        redPointY = 0
        self.height, self.width,_ = thresh.shape
        for x in range(0,self.width):
            for y in range(0,self.height):
                px = thresh[y, x]
                if 90<=int(px[2]) and (int(px[2]) - int(px[0])) > 40 and (int(px[2]) - int(px[1])) > 40:
                    redPoint+=1
                    redPointX = redPointX + x
                    redPointY = redPointY + y

        redPointX = (redPointX / redPoint)
        redPointY = (redPointY / redPoint)
        
        #Görüntüye düzeltme (undistortion) işlemleri uygulandıktan sonra, görüntü yakınlaşır. Bu haritanın ölçülerinin yanlış olmasına sebep olur, deneme yanılma metodu ile tam oturduğu çözünürlük değeri saptanır.
        #Saptanan değer mevcut çözünürlük değerine bölündüğünde çıkan değer undistortion_coefficient.

        greenPoint = 0
        greenPointX = 0
        greenPointY = 0
        for x in range(0,self.width):
            for y in range(0,self.height):
                px = thresh[y, x]
                if 60<=int(px[1]) and (px[1]) - int(px[0]) >= 20 and  (int(px[1]) - int(px[2])) >= 20:
                    #thresh[y,x] = 255,0,0
                    greenPoint+=1
                    greenPointX = greenPointX + x
                    greenPointY = greenPointY + y

        greenPointX = (greenPointX / greenPoint)
        greenPointY = (greenPointY / greenPoint)
    
        #Duvar Yüksekliğinden kaynaklı mavi noktanın mesafe kaybını giderme (duvarın izdüşümünü hesaplama)
        ######################################################
        #if(greenPointY>height/2 and greenPointX<width/2):
        if(abs(bluePointY - self.height/2) > abs(bluePointX - self.width/2)):#Tangent between center point and greenpoint
            denom = (self.width/2 - bluePointX)
            if denom == 0:#Denom : payda
                denom = 1
            tangent = -1.0*(self.height/2 - bluePointY)/denom
            
            diagonal = int(math.sqrt(2*0.10/self.resolution*0.10/self.resolution))

            counter = 0   
            for y in range(0,self.height/2):
                counter +=1
                if bluePointY < self.height/2:#Burada eğer mavi noktanın koordinatı merkezden yukarı ise işareti değiştirilir
                    y = -1*y   
                pxX_b = bluePointX + int(y*1.0/tangent)    
                pxY_b = bluePointY - y
                px1 = thresh[pxY_b,pxX_b]
                if px1[0]<50 and px1[1]<50 and px1[2]<50:# and counter > diagonal: :#px[0],px[1],px[2] = BGR B,G,R Değerlerinden herhangi biri 30dan düşükse o bölge beyaz değil demektir ve o noktaya gelene kadar
                    #px1[0],px1[1],px1[2] = 0,0,255                                 #Robotun rengi koyu olduğu için duvara gelmeden robotun üzerinde durabilir o yüzden eğer piksel sayısı robotun köşegeninin yarısından fazla olmalı
                    #thresh[(2*greenPointY - y)/2,greenPointX] = 255,0,0 #kontrol amaçlı değer
                    break
                #px1[0],px1[1],px1[2] = 0,0,255 #algoritmanın doğru çalışıp çalışmadığını kontrol etmek için çizgi çizerek kullanılabilir
                
            k = 1
            l = 1
            
            if bluePointY > self.height/2:
                l = -1
                
            if bluePointX < self.width/2:
                k = -1
                
            wall_y_b = pxY_b
            wall_x_b = pxX_b

	if(abs(bluePointY - self.height/2) < abs(bluePointX - self.width/2)):
            denom = (self.height/2 - bluePointY)
            if denom == 0:
                denom = 1 #Payda sıfır olduğu zaman bire eşitlenir
            tangent = -1.0*(self.width/2 - bluePointX)/ denom
            
            diagonal = int(math.sqrt(2*0.10/self.resolution*0.10/self.resolution))

            counter = 0

            for x in range(0,self.width/2):
                counter +=1
                if greenPointX < self.width/2:#Burada eğer mavi noktanın koordinatı merkezden yukarı ise işareti değiştirilir
                    x = -1*x     
                pxX_b = bluePointX - x
                pxY_b = bluePointY + int(x*1.0/tangent)
                px1 = thresh[pxY_b,pxX_b]
                if px1[0]<50 and px1[1]<50 and px1[2]<50:# and counter > diagonal: #px[0],px[1],px[2] = BGR B,G,R Değerlerinden herhangi biri 30dan düşükse o bölge beyaz değil demektir ve o noktaya gelene kadar
                    #px1[0],px1[1],px1[2] = 0,0,255                                 #Robotun rengi koyu olduğu için duvara gelmeden robotun üzerinde durabilir o yüzden eğer piksel sayısı robotun köşegeninin yarısından fazla olmalı
                    #thresh[(2*greenPointY - y)/2,greenPointX] = 255,0,0 #kontrol amaçlı değer
                    break
                #px1[0],px1[1],px1[2] = 0,0,255 #algoritmanın doğru çalışıp çalışmadığını kontrol etmek için çizgi çizerek kullanılabilir
            
            k = 1
            l = 1
            
            if bluePointY > self.height/2:
                l = -1
                
            if bluePointX < self.width/2:
                k = -1
                
            wall_y_b = pxY_b
            wall_x_b = pxX_b
            
        distance_b = math.sqrt(math.pow(wall_x_b - self.width/2,2) + math.pow(wall_y_b - self.height/2,2))#Mavi nokta ile merkez arasındaki yeşile en yakın duvarın merkeze uzaklığı
        
        alfa = math.atan(self.resolution*distance_b/(self.iha_height - IKA_HEIGHT)) 
        
        projection_b = math.tan(alfa)*IKA_HEIGHT
        
        bluePointX = bluePointX + k*abs(projection_b*math.sin(math.asin((wall_x_b - self.width/2)/distance_b)))
        bluePointY = bluePointY + l*abs(projection_b*math.cos(math.acos((wall_y_b - self.height/2)/distance_b)))
        
        LBBX = bluePointX - 0  + k*abs(projection_b*math.sin(math.asin((wall_x_b - self.width/2)/distance_b)))/self.resolution    # 'k' ve 'l' değerleri yeşil noktanın sol alt köşeye göre koordinatlarını ayarlamak için
        
        LBBY = -bluePointY - (-1*self.height) + l*abs(projection_b*math.cos(math.acos((wall_y_b - self.height/2)/distance_b)))/self.resolution # Sol üst köşeyi orijin olarak aldığı için bütün 'y' değerleri bunun altında kalacak ve negatif olacaktır dolayısıyla '-1' ile çarpılırlar   
 	#Duvar Yüksekliğinden kaynaklı kırmızı noktanın mesafe kaybını giderme (duvarın izdüşümünü hesaplama)
	#Duvar Yüksekliğinden kaynaklı yeşil noktanın mesafe kaybını giderme (duvarın izdüşümünü hesaplama)
	######################################################
	if(abs(greenPointY - self.height/2) > abs(greenPointX - self.width/2)):#Tangent between center point and greenpoint
            denom = (self.width/2 - greenPointX)
            if denom == 0:
                denom = 1
            tangent = -1.0*(self.height/2 - greenPointY)/denom
            
            diagonal = int(math.sqrt(2*0.10/self.resolution*0.10/self.resolution))

            counter = 0   
            for y in range(0,self.height/2): 
                counter +=1
                if greenPointY < self.height/2:#Burada eğer yeşil noktanın koordinatı merkezden yukarı ise işareti değiştirilir
                    y = -1*y   
                pxX = greenPointX + int(y*1.0/tangent)    
                pxY = greenPointY - y
                px1 = thresh[pxY,pxX]
                if px1[0]<50 and px1[1]<50 and px1[2]<50:#px[0],px[1],px[2] = BGR B,G,R Değerlerinden herhangi biri 30dan düşükse o bölge beyaz değil demektir ve o noktaya gelene kadar
                    #px1[0],px1[1],px1[2] = 0,0,255                                 #Robotun rengi koyu olduğu için duvara gelmeden robotun üzerinde durabilir o yüzden eğer piksel sayısı robotun köşegeninin yarısından fazla olmalı
                    #thresh[(2*greenPointY - y)/2,greenPointX] = 255,0,0 #kontrol amaçlı değer
                    break
                #px1[0],px1[1],px1[2] = 0,0,255 #algoritmanın doğru çalışıp çalışmadığını kontrol etmek için çizgi çizerek kullanılabilir
                
            k = 1
            l = 1
            
            if greenPointY > self.height/2:
                l = -1
                
            if greenPointX < self.width/2:
                k = -1
                
            wall_y = pxY
            wall_x = pxX

	if(abs(greenPointY - self.height/2) < abs(greenPointX - self .width/2)):
            denom = (self.height/2 - greenPointY)
            if denom == 0:
                denom = 1
            tangent = -1.0*(self.width/2 - greenPointX)/ denom
            
            diagonal = int(math.sqrt(2*0.10/self.resolution*0.10/self.resolution))

            counter = 0

            for x in range(0,self.width/2):
                counter +=1
                if greenPointX < self.width/2:#Burada eğer yeşil noktanın koordinatı merkezden yukarı ise işareti değiştirilir
                    x = -1*x     
                pxX = greenPointX - x
                pxY = greenPointY + int(x*1.0/tangent)
                px1 = thresh[pxY,pxX ]
                if px1[0]<50 and px1[1]<50 and px1[2]<50 :#px[0],px[1],px[2] = BGR B,G,R Değerlerinden herhangi biri 30dan düşükse o bölge beyaz değil demektir ve o noktaya gelene kadar
                    #px1[0],px1[1],px1[2] = 0,0,255                                 #Robotun rengi koyu olduğu için duvara gelmeden robotun üzerinde durabilir o yüzden eğer piksel sayısı robotun köşegeninin yarısından fazla olmalı
                    #thresh[(2*greenPointY - y)/2,greenPointX] = 255,0,0 #kontrol amaçlı değer
                    break
                #px1[0],px1[1],px1[2] = 0,0,255 #algoritmanın doğru çalışıp çalışmadığını kontrol etmek için çizgi çizerek kullanılabilir
            
            k = 1
            l = 1
            
            if greenPointY > self.height/2:
                l = -1
                
            if greenPointX < self.width/2:
                k = -1
                
            wall_y = pxY
            wall_x = pxX
            
        distance = math.sqrt(math.pow(wall_x - self.width/2,2) + math.pow(wall_y - self.height/2,2))#Yeşil nokta ile merkez arasındaki yeşile en yakın duvarın merkeze uzaklığı
        
        alfa = math.atan(self.resolution*distance/(self.iha_height - 0.4)) 
        
        projection = math.tan(alfa)*0.4
        
        LBGX = greenPointX - 0  + k*abs(projection*math.sin(math.asin((wall_x - self.width/2)/distance)))/self.resolution    # 'k' ve 'l' değerleri yeşil noktanın sol alt köşeye göre koordinatlarını ayarlamak için
        
        LBGY = -greenPointY - (-1*self.height) + l*abs(projection*math.cos(math.acos((wall_y - self.height/2)/distance)))/self.resolution
        
        # Sol üst köşeyi orijin olarak aldığı için bütün 'y' değerleri bunun altında kalacak ve negatif olacaktır dolayısıyla '-1' ile çarpılırlar   
        
        self.green_originX = -LBGX
        self.green_originY = -LBGY
 	#Duvar Yüksekliğinden kaynaklı kırmızı noktanın mesafe kaybını giderme (duvarın izdüşümünü hesaplama)
	######################################################           
	if(abs(redPointY - self.height/2) > abs(redPointX - self.width/2)):
            tangent_r = -1*(self.height/2 - redPointY)/(self.width/2 - redPointX) #Tangent between center point and redpoint
            
            diagonal = int(math.sqrt(2*0.10/self.resolution*0.10/self.resolution))
            counter = 0

            for y in range(0,self.height/2):
                counter +=1
                if redPointY < self.height/2:#Burada eğer yeşil noktanın koordinatı merkezden yukarı ise işareti değiştirilir
                    y = -1*y
                pxX = redPointX + int(y*1.0/tangent_r)
                pxY = redPointY - y
                px1 = thresh[pxY,pxX]
                if px1[0]<50 and px1[1]<50 and px1[2]<50:#px[0],px[1],px[2] = BGR B,G,R Değerlerinden herhangi biri 30dan düşükse o bölge beyaz değil demektir ve o noktaya gelene kadar
                    #px1[0],px1[1],px1[2] = 0,0,255                                 #Robotun rengi koyu olduğu için duvara gelmeden robotun üzerinde durabilir o yüzden eğer piksel sayısı robotun köşegeninin yarısından fazla olmalı
                    #thresh[(2*greenPointY - y)/2,greenPointX] = 255,0,0 #kontrol amaçlı değer
                    break
                #px1[0],px1[1],px1[2] = 0,0,255 #algoritmanın doğru çalışıp çalışmadığını kontrol etmek için çizgi çizerek kullanılabilir
                
            k = 1
            l = 1
            
            if redPointY > self.height/2:
                l = -1
                
            if redPointX < self.width/2:
                k = -1
                
            wall_y_r = pxY
            wall_x_r = pxX

	if(abs(redPointY - self.height/2) < abs(redPointX - self.width/2)):
            tangent_r = -1*(self.width/2 - redPointX)/(self.height/2 - redPointY)
            
            diagonal = int(math.sqrt(2*0.10/self.resolution*0.10/self.resolution))
            counter = 0
            for x in range(0,self.width/2):
                counter +=1
                if redPointX < self.width/2:#Burada eğer yeşil noktanın koordinatı merkezden yukarı ise işareti değiştirilir
                    x = -1*x  
                pxX_r = redPointX - x
                pxY_r = redPointY + int(x*1.0/tangent_r)
                px1 = thresh[pxY_r,pxX_r ]
                if px1[0]<50 and px1[1]<50 and px1[2]<50:#px[0],px[1],px[2] = BGR B,G,R Değerlerinden herhangi biri 30dan düşükse o bölge beyaz değil demektir ve o noktaya gelene kadar
                    #px1[0],px1[1],px1[2] = 0,0,255                                 #Robotun rengi koyu olduğu için duvara gelmeden robotun üzerinde durabilir o yüzden eğer piksel sayısı robotun köşegeninin yarısından fazla olmalı
                    #thresh[(2*greenPointY - y)/2,greenPointX] = 255,0,0 #kontrol amaçlı değer
                    break
                #px1[0],px1[1],px1[2] = 0,0,255 #algoritmanın doğru çalışıp çalışmadığını kontrol etmek için çizgi çizerek kullanılabilir
                
            k = 1
            l = 1
            
            if redPointY > self.height/2:
                l = -1
                
            if redPointX < self.width/2:
                k = -1
                
            wall_y_r = pxY
            wall_x_r = pxX
            
        distance_r = math.sqrt(math.pow(wall_x_r - self.width/2,2) + math.pow(wall_y_r - self.height/2,2)) #Kırmızı nokta ile merkez arasındaki kırmızıya en yakın duvarın merkeze uzaklığı
        
        alfa_r = math.atan(self.resolution*distance_r/(self.iha_height - 0.4)) 
        
        projection_r = math.tan(alfa_r)*0.4
        #Bu bölümde bulunan izdüşüm(projection) değeri x ve y bileşenlerine ayrılarak bu değerler kırmızı noktanın koordinatlarına eklenir
        #Ardından kırmızı noktanın koordinatları sol alt köşe referans alınarak yeniden belirlenir
        LBRX = redPointX - 0  + k*abs(projection_r*math.sin(math.asin((wall_x_r - self.width/2)/distance_r)))/self.resolution    
        
        LBRY = -redPointY - (-1*self.height) + l*abs(projection_r*math.cos(math.acos((wall_y_r - self.height/2)/distance_r)))/self.resolution # Sol üst köşeyi orijin olarak aldığı için bütün 'y' değerleri bunun altında kalacak ve negatif olacaktır
	
        self.RVIZXR = LBRX - LBGX  # yeşil nokta RVIZ'de(map_server) orijin (0,0) olarak alınacağı için kırmızı noktanın 'x' ve 'y' değerleri bu nokta referans alınarak tekrar düzenlenir
        self.RVIZYR = LBRY - LBGY
        self.RVIZXB = -LBBX
        self.RVIZYB = -LBBY
        
        self.RVIZXG = LBGX-LBBX  #Yeşil noktanın koordinatları kadar harita ters tarafa hareket ettirilir böylelikle yeşil nokta (ika) tam (0,0) noktasında oluyoruz
        self.RVIZYG = LBGY-LBBY
        
        self.redGoalX = redPointX  + k*abs(projection_r*math.sin(math.asin((wall_x_r - self.width/2)/distance_r)))/self.resolution 
        
        self.redGoalY = redPointY + l*abs(projection_r*math.cos(math.acos((wall_y_r - self.height/2)/distance_r)))/self.resolution
        
        self.greenGoalX = greenPointX  + k*abs(projection*math.sin(math.asin((wall_x - self.width/2)/distance)))/self.resolution    # 'k' ve 'l' değerleri yeşil noktanın sol alt köşeye göre koordinatlarını ayarlamak için
        
        self.greenGoalY = greenPointY + l*abs(projection*math.cos(math.acos((wall_y - self.height/2)/distance)))/self.resolution
        
        if self.map_flag == 2:
            cv2.drawContours(thresh,[box],0,(0,0,0),int(WALL_THICKNESS/self.resolution)) # Haritayı ikini kez oluştururken 
        
        cv2.circle(thresh,(int(self.redGoalX),int(self.redGoalY)),int(INFLATION_LAYER/self.resolution), (255,255,255), -1)
        
        cv2.circle(thresh,(int(self.greenGoalX),int(self.greenGoalY)),int(INFLATION_LAYER/self.resolution), (255,255,255), -1)
        cv2.imwrite('/home/nvidia/ws/src/hector/hector_ika/maps/map.png',thresh)

        return thresh

    def map_creator(self, thresh): #Bu fonksiyon kırpılmış resmi alıp bir .pgm ve .yaml (Map) oluşturuyor.
        current_seyyah_yaw = self.seyyahTheta
        image = "map.pgm"
        name = "map.yaml"
        #ika ve labirentin duvarı arasındaki yeşil bölgeyi beyaza boyayarak haritalama esnasında duvarla robotun birbirine kaynaşmasını engelliyor
        gray = cv2.cvtColor(thresh, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(gray, WALL_THRESHOLD, 255, cv2.THRESH_BINARY_INV)
        _, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
        
                                                                #En ve boydan hangisinin piksel değeri daha küçükse o piksel hesaplama için kullanılır
                                                                #hfov d eğeri olan 130'un yarısı olan 65(1.134464014 radyan) derecenin tanjantı ile
                                                                #ihanın o anki yüksekli değerinin çarpımının iki katı bize kameranın bir ucundan diğerine olan
                                                                #metre cinsinden uzaklığı verir. Bunu resmin(640x480) genişlik değerine yani 640'a bölerek
                                                                #çözünürlük değeri bulunur
       
        #--     Gürültü temizleme bölümü
        
        length = []                                             #Kontür uzunluklarını depolamak için bir array oluşturulur
        for i in range(len(contours)):#
                length.insert(i,cv2.arcLength(contours[i],True))

        for k in range(len(contours)):
                if length[k]*self.resolution <=MIN_NOISE_CIRCUMFERENCE:#Kontürün uzunluk değerini çözünürlük değeri ile çarparak metre cinsinden uzunlukları bulunur. Etrafını sardıkları nesnenin boyu 1.8 metreden küçük ise bself.unu beyaza boyar.
                        cv2.drawContours(thresh, contours, k, (255,255,255),-1)#Kontür kalınlığının negatif değer almasının sebebi nesnenin etrafını sarmak yerine onu boyamasını sağlamak
        cv2.imwrite('./Gurultu_Temizleme.png',thresh)#Pgm dosyası kayıt edilir
        ret, thresh = cv2.threshold(thresh, CLEARING_THRESHOLD, 255, cv2.THRESH_BINARY_INV)# Ve griye boyanmış gürültü nesneleri silinir

        cv2.imwrite('/home/nvidia/ws/src/hector/hector_ika/maps/map.pgm',thresh)#Pgm dosyası kayıt edilir

        f = open('/home/nvidia/ws/src/hector/hector_ika/maps/%s'%name,"w+")#Yaml dosyası oluşturulur
        f.write("image: %s\r\n" % image)
        f.write("resolution: %.10f\r\n" % self.resolution)
        if self.map_flag == 1:
            f.write("origin: ['%.10f', '%.10f', %.10f]\r\n"% (self.RVIZXB*self.resolution  , self.RVIZYB*self.resolution,0.0000))
            
        if self.map_flag == 2:
            f.write("origin: ['%.10f', '%.10f', 0.000000]\r\n"% (self.green_originX*self.resolution  , self.green_originY*self.resolution))
            
        f.write("negate: 0\r\n") # for invers thresh negate: 1
        f.write("occupied_thresh: 0.94\r\n")
        f.write("free_thresh: 0.038\r\n")
        f.close()
        quat = tf.transformations.quaternion_from_euler(0, 0, self.seyyahInitTheta + PI/2.0) # burada "PI" eklememizin sebebi map_server'da haritanın 90 derece sola dönük başlaması
        
	self.goal = MoveBaseGoal()
	self.goal.target_pose.header.stamp = rospy.Time.now()
	self.goal.target_pose.header.frame_id = "map"	

        if self.map_flag == 1:
            self.goal.target_pose.pose.orientation.x = quat[0]
            self.goal.target_pose.pose.orientation.y = quat[1]
            self.goal.target_pose.pose.orientation.z = quat[2]
            self.goal.target_pose.pose.orientation.w = quat[3]
            self.goal.target_pose.pose.position.x = self.RVIZXG*self.resolution 
            self.goal.target_pose.pose.position.y = self.RVIZYG*self.resolution
        
        if self.map_flag == 2:
            self.goal.target_pose.pose.orientation.x = quat[0]
            self.goal.target_pose.pose.orientation.y = quat[1]
            self.goal.target_pose.pose.orientation.z = quat[2]
            self.goal.target_pose.pose.orientation.w = quat[3]
            self.goal.target_pose.pose.position.x = self.RVIZXR*self.resolution 
            self.goal.target_pose.pose.position.y = self.RVIZYR*self.resolution
            
            
            
def main():
    time.sleep(2)
    rospy.init_node("uav_control_node", anonymous=True)   
    robot = RobotManager()
    robot.droneCode_setup()
    robot.arm_and_takeoff(TAKEOFF_HEIGHT)
    robot.rate.sleep()
    while True:
        robot.gozcu_mover()#Goal height is 6 meters
        instant_resolution = 2.0*robot.getALT()*math.tan(HFOV/2.0)/(WIDTH_RATIO*UNDISTORTION_COEFFICIENT)
        if robot.getALT()>TAKEOFF_HEIGHT and abs(robot.t - robot.b)*math.pow(instant_resolution,2) < 0.5 and abs(robot.r - robot.l)*math.pow(instant_resolution,2)<0.5:
            robot.GoPro.publish(1) 
            photographing_height = robot.getALT()
            print'photographing_height',photographing_height
            time.sleep(10)
            robot.map_creator(robot.image_croper(photographing_height,robot.GoPro_image))
            os.system("bash /home/nvidia/ws/src/hector/hector_ika/src/Secondary_Map.sh")
            time.sleep(20)
            print('Map is created')
            cv2.imwrite('./GoPro_image.png',robot.GoPro_image)
            break
    print("specialRTL active")
    robot.specialRTL()
    robot.move_base.send_goal(robot.goal)
    while robot.move_base_status != "Goal reached." or robot.move_base_status == None:
        robot.rate.sleep()
        
    robot.map_flag = 2
    robot.map_creator(robot.image_croper(photographing_height,robot.GoPro_image))
    os.system("bash /home/nvidia/ws/src/hector/hector_ika/src/Secondary_Map.sh")#Yeni harita için tekrar map_server'i başlatır
    time.sleep(20)    
    
    robot.move_base.send_goal(robot.goal)
    
    print("Mission Completed!")
    
if __name__ == "__main__":
    main()
