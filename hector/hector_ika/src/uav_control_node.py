#!/usr/bin/env python
# -*- coding: utf-8 -*-

#YAZGİT - Yapay Zeka ve Grüntü İşleme Topluluğu
wall_interval = 0.7

#OpenCV Kütüphaneleri
import numpy as np
import cv2
import cv_bridge

#ROS Kütüphaneleri
import rospy #Python için ros kütüphanesi
from sensor_msgs.msg import Image,Imu #İKA ve İHAdan yön verisi alınmasını sağlar
from geometry_msgs.msg import Twist, PoseStamped,WrenchStamped,Point
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
#from nav_msgs.msg import Odometry
import actionlib, tf
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import euler_from_quaternion # imu sensöründen "quaternion" şeklinde alınan veriye "euler" formatına çevirmek için kullanılır
#from hector_uav_msgs.msg import LandingActionGoal
#from actionlib_msgs.msg import GoalStatusArray

#Standart Python Kütüphaneleri
import tf
import math
import time
import os
import sys
import subprocess

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
        #Veri havuzu
        self.downcam_image_data = None
        self.ugv_odom = None
        self.uav_odom = None
        self.rate = rospy.Rate(10) # sabit bir duraksama süresi belirler (1/10) saniye
        self.uavPoseFlag = 0 # İHA'nın kalktığı ilk pozsiyonu kaydetmek için kullanılır. IHA'dan ilk odometre verisi geldiği anda değer '1' olarak değişir ve ilk değer kalkış yerinin konumu olarak saklanır.
        self.goHomeFlag = 0 # İHA fotoğraf çektikten sonra değer '1' olarak değiştirilir, değer '1' olduğu anda iniş başlar.
        #Kamera Kalibrasyon Değerleri
        #Kameradaki balıkgözü (fisheye) etkisini silmek için kullanılan python kodunun verdiği değerler
        
        #Gazebo 800x600 kamera için kalibrasyon değerleri
        
        self.DIM=(800, 600)
        self.K=np.array([[337.7289115144896, 0.0, 399.713530794396], [0.0, 337.7056705555252, 298.3935207374622], [0.0, 0.0, 1.0]]) #Kamera Kalibrasyonu sonucu ortaya çıkan değerler
        self.D=np.array([[0.07785624707613907], [0.03817987944876708], [-0.061388108802514295], [0.04096004118965009]])
        self.balance=0.0
        self.dim2=None
        self.dim3=None
        
        #################
        
        #Gazebo 320x240 kamera için kalibrasyon değerleri
        """
        self.DIM=(800, 600)
        self.K=np.array([[31383.869382481396, 0.0, 399.56353239198387], [0.0, 31390.860528189492, 299.17345946153046], [0.0, 0.0, 1.0]])
        self.D=np.array([[-1272.8133226612754], [897123.6189387914], [-517407383.9392094], [166604841065.6171]])
        self.dim2=None
        self.dim3=None
        self.balance=0.0
        """
        self.ugv_cam_min_hsv_limit = np.array([0,83,103])
        self.ugv_cam_max_hsv_limit = np.array([186,255,255])
        self.ugv_cam_flag = 0
        self.uav_loc_flag = 0
        self.width_ratio = 800 #Kamera görüntüsünün yataydaki piksel sayısı
        self.undistortion_coefficient = 1.336150798
        self.hfov = 1.069886831 #Radyan cinsinde kameranın hfov(yataydaki görüş açısı) değeri
        self.time1 = 0
        self.time2 = 0
        self.velx_iha1 = 0
        self.velx_iha2 = 0
        self.vely_iha1 = 0
        self.vely_iha2 = 0
        self.xDist = 0 # İHAnın 'x' ekseninde katettiği piksel mesafesi
        self.yDist = 0 # İHAnın 'y' ekseninde katettiği piksel mesafesi
        #Hector hız değişkeni
        self.vel_hec = Twist() #IHA'nın hız değeri tanımlanır
        self.vel_ugv = Twist() #İKA'nın hız değeri tanımlanır
        self.blue_y = self.DIM[0]/2
        self.blue_x = self.DIM[1]/2
        #cv_bridge rosimgmsg'yi cv2 img'ye dönüştürür
        self.ugv_c = None
        self.img_loc = None
        self.cv_bridge = cv_bridge.CvBridge()
        self.map_flag = 1
        self.move_base_status = None
        self.movebase_callback_flag = 0
        self.status = None #Move base status
        self.ugv_imu_flag = 0
        #self.wall_interval = 0.7
        self.wall_thickness = 0.05
        #Actionlib Bölümü
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
        #Subscribe bölümü
        #İKA ve IHA'nın (Hector) imularına yönlerini eşitlemek amacıyla üye oluyoruz,
        rospy.Subscriber("/clock",Clock,self.clock_callback) #Ros zamanı ihanın kalkış noktasını işaretledikten sonra hareket ettirmek için gerekli
        rospy.Subscriber("/iha/raw_imu",Imu,self.uav_imu_callback)
        rospy.Subscriber("/imu",Imu,self.ugv_imu_callback)
        rospy.Subscriber("/cmd_vel",Twist,self.ugv_vel_callback) # İKAnın ekranında harekete başladıktan sonra 'Following the path' yazısı yazması için üye olunur şart değil
        #rospy.Subscriber("/move_base/status",GoalStatusArray,self.goal_status_callback)
        
        #İHA'dan odometre verisi alınır
        self.uav_odom = rospy.Subscriber("/iha/ground_truth_to_tf/pose", PoseStamped, self.uav_odom_callback, queue_size=1)
        self.uav_state = rospy.Subscriber("/iha/ground_truth/state", Odometry, self.uav_state_callback, queue_size=1)
        
        #IHA ve İKA'nın kamera verileri alınır
        self.uav_downcam_image = rospy.Subscriber("/iha/downward_cam/camera/image", Image, self.downcam_image_callback, queue_size=1)
        rospy.Subscriber("/ika/camera1/image_raw",Image,self.ugv_image_callback)
        rospy.Subscriber("/move_base/status",GoalStatusArray,self.move_base_status_callback)
        #Publish bölümü
        #İKA ve IHA'ya hareket emirlerini vereceğiz
        self.uav_cmd_vel = rospy.Publisher("/iha/cmd_vel", Twist, queue_size=1)
        self.ugv_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.seyyah_cmd_vel = rospy.Publisher("/seyyah/cmd_vel", Twist, queue_size=1)

        #map_server'a göndermek için belirlenen yeşil nokta(başlangıç noktaları)
        self.RVIZXG=0
        self.RVIZYG=0
        #rospy.spin()


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


    def uav_state_callback(self,msg):
        self.uav_inst_vel = msg.twist.twist
        self.uavInstVelX = self.uav_inst_vel.linear.x
        self.uavInstVelY = self.uav_inst_vel.linear.y
        self.uavInstVelTheta = self.uav_inst_vel.angular.z
        #print 'self.uav_inst_vel',self.uav_inst_vel

    def clock_callback(self,msg):
        self.rostime = msg.clock.secs
        #print self.rostime.secs
        #print self.vel_hec.linear

    def ugv_vel_callback(self,msg): #İKA'nın hız verisi alınır
        self.ugv_vel_x = msg
        self.seyyah_cmd_vel.publish(msg)

    def ugv_image_callback(self,msg):
            
        self.ugv_image_data = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        
        
    def downcam_image_callback(self, msg): #cv_bridge rosimgmsg'yi cv2 img'ye dönüştürür
    
      try:
          self.downcam_image_data = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
          #self.downcam_image_data = cv2.resize(self.downcam_image_data, (800, 600))
          while self.downcam_image_data is None:
              self.rate.sleep()  
          self.img = self.downcam_image_data
          self.dim1 = self.img.shape[:2][::-1]  #dim1 is the dimension of input image to un-distort
          assert self.dim1[0]/self.dim1[1] == self.DIM[0]/self.DIM[1], "Image to undistort needs to have same aspect ratio as the ones used in calibration"
          if not self.dim2:
            self.dim2 = self.dim1
            #print 'asdasd'
          if not self.dim3:
            self.dim3 = self.dim1

          scaled_K = self.K * self.dim1[0] / self.DIM[0]  # The values of K is to scale with image dimension.
          scaled_K[2][2] = 1.0  # Except that K[2][2] is always 1.0
          ## This is how scaled_K, dim2 and balance are used to determine the final K used to un-distort image. OpenCV document failed to make this clear!
          new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(scaled_K, self.D, self.dim2, np.eye(3), balance=self.balance)
          map1, map2 = cv2.fisheye.initUndistortRectifyMap(scaled_K, self.D, np.eye(3), new_K, self.dim3, cv2.CV_16SC2)
          self.undistorted_img = cv2.remap(self.img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
      except cv_bridge.CvBridgeError:
          return
      self.img_cam = self.undistorted_img
      #self.img_cam = self.downcam_image_data

      self.img_cam = cv2.cvtColor(self.img_cam, cv2.COLOR_BGR2GRAY)
      ret, self.img_cam = cv2.threshold(self.img_cam, 100, 255, cv2.THRESH_BINARY_INV)
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
      cv2.putText(self.img_cam,('Height : %.2f' %(self.uav_odom[2])),(5,20),font,0.5,(255,255,255),1,cv2.LINE_AA)
      cv2.putText(self.img_cam,('Vel x : %.2f' %(self.vel_hec.linear.x)),(5,40),font,0.5,(255,255,255),1,cv2.LINE_AA)
      cv2.putText(self.img_cam,('Vel y : %.2f' %(self.vel_hec.linear.y)),(5,60),font,0.5,(255,255,255),1,cv2.LINE_AA)
      cv2.putText(self.img_cam,('Vel z : %.2f' %(self.vel_hec.linear.z)),(5,80),font,0.5,(255,255,255),1,cv2.LINE_AA)
      cv2.putText(self.img_cam,('Vel theta : %.2f' %(self.vel_hec.angular.z)),(5,100),font,0.5,(255,255,255),1,cv2.LINE_AA)
      cv2.putText(self.img_cam,('Area Top : %.2f' %(self.t)),(cols/2-100,20),font,0.5,(255,255,255),1,cv2.LINE_AA)
      cv2.putText(self.img_cam,('Area Bottom : %.2f' %(self.b)),(cols/2-100,rows-10),font,0.5,(255,255,255),1,cv2.LINE_AA)
      cv2.putText(self.img_cam,('Area Left : %.2f' %(self.l)),(3,rows/2+5),font,0.5,(255,255,255),1,cv2.LINE_AA)
      cv2.putText(self.img_cam,('Area Right : %.2f' %(self.r)),(cols-180,rows/2+5),font,0.5,(255,255,255),1,cv2.LINE_AA)

    def uav_imu_callback(self,msg):#İHAnın yön verisi alınır
        self.uav_imu = msg.orientation
        rot = self.uav_imu
        (roll_0,pitch_0,self.uav_theta) = euler_from_quaternion ([rot.x,rot.y,rot.z,rot.w])
        self.uav_theta = self.uav_theta - 1.570796327 #Resmi map server'a gönderirken 90 derece sağa çevirmek gerekiyor.
                                                     #Resim çekildikten sonra dödürmek bozulmalara sebep verdiği için,
                                                     #ihanın "theta" değerini manipüle ederek ikanın "theta değerine göre"
                                                     #90 derece sağa dönük olarak resim çekmesini sağlıyoruz
                                                     
       #IMU -1,57,+1,57 radyan (-180 + 180 derece) aralığında açı verisi dönüyor. Negatif değerlerden kurtulmak için imu verisini 0-3,14 radyan (0-360 derece) şeklinde yazarız
        self.uav_theta = self.uav_theta + 3.141592654
        if self.uav_theta > 6.283185308:
            self.uav_theta = self.uav_theta - 6.283185308

    def ugv_imu_callback(self,msg):##İKAnın yön verisi alınır
        self.ugv_imu = msg.orientation
        self.ugv_rot = self.ugv_imu
        (roll_0,pitch_0,self.ugv_theta) = euler_from_quaternion ([self.ugv_rot.x,self.ugv_rot.y,self.ugv_rot.z,self.ugv_rot.w])
        self.def_ugv_theta = self.ugv_theta 
        if self.ugv_imu_flag == 0:    
            self.init_ugvTheta = self.ugv_theta
            self.ugv_imu_flag = 1
        self.ugv_theta = self.ugv_theta + 3.141592654
        if self.ugv_theta > 6.283185308:
            self.ugv_theta = self.ugv_theta - 6.283185308

    def ugv_odom_callback(self, msg): #Quaternion yerine euler kullanıyoruz
      pose = msg.pose.pose

      quat = (
          pose.orientation.x,
          pose.orientation.y,
          pose.orientation.z,
          pose.orientation.w,
      )
      orientation = tf.transformations.euler_from_quaternion(quat)
      self.ugv_odom = (
          pose.position.x,
          pose.position.y,
          orientation[2]
      )

    def uav_odom_callback(self, msg,height_tolerance = 0.5,const = 0.1):  #quaternion yerine euler kullanıyoruz
      pose = msg.pose
      if(self.uavPoseFlag == 0):
        self.uavFirstPose = msg.pose
        self.uavPoseFlag = 1
      quat = (
          pose.orientation.x,
          pose.orientation.y,
          pose.orientation.z,
          pose.orientation.w,
      )
      orientation = tf.transformations.euler_from_quaternion(quat)
      self.uav_odom = (
          pose.position.x,
          pose.position.y,
          pose.position.z,
          orientation[2]
      )

      if(self.goHomeFlag == 1):#Kalkılan yere geri iniş 
        if (self.uav_odom[2] - self.uavFirstPose.position.z >= height_tolerance):
          self.vel_hec.linear.x = const*(self.uav_odom[0] - self.uavFirstPose.position.x)
          self.vel_hec.linear.y = const*(self.uav_odom[1] - self.uavFirstPose.position.y)
          self.vel_hec.linear.z = const*(self.uavFirstPose.position.z - self.uav_odom[2])
          self.uav_cmd_vel.publish(self.vel_hec)

      self.rate.sleep()
    #def ugv_mover(self,vel_x = 0.05,const_ang = 0.1):
        #self.rate.sleep()
        #while self.ugv_c == None:
            #self.rate.sleep()
        #while self.ugv_c >0:
            #cv2.imshow('self.ugv_cam',self.ugv_image_data)
            #cv2.waitKey(100)
            #self.vel_ugv.linear.x = vel_x
        
            #if self.ugv_r >= self.ugv_l:
                #self.vel_ugv.angular.z = const_ang*(self.ugv_l - self.ugv_r)/(self.ugv_l + 0.0001)

            #if self.ugv_l > self.ugv_r:
                #self.vel_ugv.angular.z = const_ang*(self.ugv_l - self.ugv_r)/(self.ugv_r + 0.0001)
            #if self.ugv_cam_flag != 3:
                #self.ugv_cmd_vel.publish(self.vel_ugv)
            #self.rate.sleep()
            
        #self.vel_ugv.linear.x = 0
        #self.vel_ugv.angular.z = 0
        #if self.ugv_cam_flag != 3:
            #self.ugv_cmd_vel.publish(self.vel_ugv)
        #cv2.destroyAllWindows()


    def ugv_yaw_equalizer(self,const = 0.6,tolerance = 0.03):
        while abs(self.init_ugvTheta - self.def_ugv_theta) > tolerance:     
            self.vel_ugv.angular.z = (self.init_ugvTheta - self.def_ugv_theta)*const
            self.ugv_cmd_vel.publish(self.vel_ugv)
            print 'Yaw Equaling'
            
        self.vel_ugv.angular.z = 0    
        self.ugv_cmd_vel.publish(self.vel_ugv)    
            
            

    def hector_mover(self,height,const_lin = 0.1,const_ang = 0.52,height_tolerance = 0.1,angel_tolerance = 0.087266463):# Hectorun kalkışından sorumlu, Yön eşitleme kısmı gözden geçirilmeli
                                                                                                                        # Döndürmeye Gerek kalmayabilir
                                                                                                                        
        while self.downcam_image_data is None:
            self.rate.sleep()
        self.rate.sleep()   
        self.vel_hec.angular.z = -1*const_ang*(self.uav_theta - self.ugv_theta)

        if abs(round(height - self.uav_odom[2],2))>=height_tolerance:
            self.vel_hec.linear.z = const_lin*(height - self.uav_odom[2])
            if(self.vel_hec.linear.z<0):# Aşağı inerken çok yavaş indiği için
              self.vel_hec.linear.z = 3*self.vel_hec.linear.z

        #print 'UAV',self.uav_theta*57.295779513
        #print 'UGV',self.ugv_theta*57.295779513

        #self.t görüntünün ortadan üst kısmı, self.b alt kısmı, self.r sağ ve self.l sol kısmındaki beyaz alanların büyüklüğünü temsil eder.
        #Bu alaları resmin dört tarafında da eşit olmasını sağlamaya çalışarak ihaya hız komutu verir.

        #Büyük alanı küçük alana bölerek aradaki kat farkına göre hız komutu yayınlanır. 0.0001 sabitinin sebebi küçük alanın (paydanın) sıfır olması durumunda programın durmasını engellemek.
        if(self.uav_odom[2]>height/2):
        
            if self.t > self.b:
                self.vel_hec.linear.x = 1.5*const_lin*(self.t - self.b)/(self.b + 0.0001)*2.0

            if self.t == self.b:
                self.vel_hec.linear.x = 0.0

            if self.b >self.t:
                self.vel_hec.linear.x = 1.5*const_lin*(self.t - self.b)/(self.t + 0.0001)*2.0

            if self.r > self.l:
                self.vel_hec.linear.y = 1.5*const_lin*(self.l - self.r)/(self.l + 0.0001)*2.0

            if self.r == self.l:
                self.vel_hec.linear.y = 0.0
            
            if self.l > self.r:
                self.vel_hec.linear.y = 1.5*const_lin*(self.l - self.r)/(self.r + 0.0001)*2.0

            ##Eğer alanlar arasındaki fark devasa ise aşırı büyük hız komutları yayınlanacaktır.
            ##Bunu engellemek için eğer hız değeri belirli bir eşiğin üzerinde ise (0.5 sembolik) onu sabit bir sayıya sabitleyerek ihanın daha stabil davranması sağlanır
            if self.vel_hec.linear.x >= 0.5:
                self.vel_hec.linear.x = 0.5

            if self.vel_hec.linear.x < -0.5:
                self.vel_hec.linear.x = -0.5

            if self.vel_hec.linear.y >= 0.5:
                self.vel_hec.linear.y = 0.5

            if self.vel_hec.linear.y < -0.5:
                self.vel_hec.linear.y = -0.5

        self.uav_cmd_vel.publish(self.vel_hec)
        
        #if self.time1 == 0:
            #self.time1 = self.time2
        
        #if self.time2 - self.time1 >= time_int:
            #self.instant_resolution = 2.0*self.uav_odom[2]*math.tan(self.hfov)/(self.width_ratio*self.undistortion_coefficient) #self.uav_odom[2] değeri ihanın yükseklik değeridir
            #self.velx_iha2 = self.uavInstVelX
            #self.vely_iha2 = self.uavInstVelY
            #self.xDist = ((self.velx_iha1 + self.velx_iha2)/2)*((self.time2 - self.time1))/(2.0*(self.uav_odom[2])*math.tan(self.hfov)/(self.width_ratio*self.undistortion_coefficient))
            #self.yDist = ((self.vely_iha1 + self.vely_iha2)/2)*((self.time2 - self.time1))/(2.0*(self.uav_odom[2])*math.tan(self.hfov)/(self.width_ratio*self.undistortion_coefficient))
            #print 'instant_resolution :',(2.0*(self.uav_odom[2])*math.tan(self.hfov)/(self.width_ratio*self.undistortion_coefficient))
            #print 'self.velx_iha1, self.velx_iha2',self.velx_iha1, self.velx_iha2
            ##print 'self.vely_iha2',self.vely_iha2
            #print 'X',self.xDist
            #print 'Y',self.yDist
            #self.velx_iha1 = self.velx_iha2
            #self.vely_iha1 = self.vely_iha2
            #print 'self.time2 - self.time1',self.time2 - self.time1
            #self.time1 = self.time2
            #rows,cols,channels = self.img_loc.shape
            
            #self.blue_x = self.blue_x + int(self.xDist)
            #self.blue_y = self.blue_y + int(self.yDist)
            
            #cv2.circle(self.img_loc,(self.blue_y,self.blue_x), 5, (255,0,0), -1)
                    
                #self.rate.sleep()

    def is_there_red_and_green(self):#Yeşil ve Kırmızıyı aynı anda görüp görmediğini kontrul eder(hsv limit değerlerinin değiştirilmesi gerekebilir
        red_flag = False
        green_flag = False

        red_min_hsv_limit = np.array([0,1,0])
        red_max_hsv_limit = np.array([54,255,255])

        green_min_hsv_limit = np.array([1,0,0])
        green_max_hsv_limit = np.array([255,255,255])

        self.rate.sleep()

        img = self.downcam_image_data

        hsv_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        img_red = hsv_image
        img_green = hsv_image

        img_red = cv2.inRange(img_red, red_min_hsv_limit, red_max_hsv_limit)
        img_green = cv2.inRange(img_green, green_min_hsv_limit, green_max_hsv_limit)

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
        #self.undistortion_coefficient = 1.336150798
        self.resolution = 2.0*iha_height*math.tan(1.069886831)/(self.width_ratio*self.undistortion_coefficient)
        
        img_maze = photograph
        h,w,_ = img_maze.shape
        self.iha_height = iha_height
        print 'resize'
        cv2.imwrite('%s/catkin_ws/src/hector/hector_ika/maps/img_maze.png'%os.path.expanduser('~'),img_maze)
        
        self.goHomeFlag = 1 # Resim artık çekildiği için iha inişe geçebilir bunu goHomeFlag değerini 1 yaparak sağlarız
        
        gray = cv2.cvtColor(img_maze, cv2.COLOR_BGR2GRAY)

        ret, thresh = cv2.threshold(gray, 90, 255, cv2.THRESH_BINARY_INV)
        #kernel = np.ones((8, 8), np.uint8)#Çerçeveyi bir miktar genişleterek tüm labirentin alınmasını kolaylaştırır
        
        dilation_kernel_dim = int((wall_interval) / self.resolution) # Çerçeve bir miktar geniş olsun diye duvarları 0.3 metre kadar kalınlaştırıyoruz

        closing_kernel_dim = int((wall_interval + 0.1) / self.resolution) # Close işlminde kullnılacak matrisin
                                            # satır ve sütün sayıları gerçekte 0.3 metreye tekabul
                                            # edecek şekilde ayarlanır
                                            # 0.3 metre de kalınlaştırma olduğu için labirent tek blok haline gelir ve çerçeve hatasız bir şekilde oluşturulur

        print 'closing_kernel_dim  ',closing_kernel_dim 

        closing_kernel = np.ones((closing_kernel_dim , closing_kernel_dim ), np.uint8)

        dilation_kernel = np.ones((dilation_kernel_dim, dilation_kernel_dim), np.uint8)

        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, closing_kernel,iterations=1) # Kapatma işlemi
        
        thresh_dilated = cv2.dilate(thresh, dilation_kernel, iterations=1) # Genişletme işlemi
        
        #Bu iki işlem sonunda labirent binary image'e dönüştüğünde tek bir beyaz blok'a dönüşür ve çerçeve oluşturmak daha kolay olur

        #Geniş bir çerçeve çizmek için
        #Bir duvar aralığının yarısı kadar genişletilmiş, görüntüdeki en büyük kontür bulunur
        #(yani çerçeve)
        thresh, contours,hierarchy = cv2.findContours(thresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

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

        #rect = cv2.minAreaRect(cnt)
        
        rect = cv2.minAreaRect(cnt)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        (mx,my),(mw,mh),angle = rect
        
        mw = (mw + 0.3) / self.resolution #Çerçeve olarak çizilecek olan dikdörtgeni 0.3 metre genişletiyoruz ki noktaları hedef gösterirken sorun yaratmasın
        mh = (mh + 0.3) / self.resolution

        print 'box',box
    
        #mxV = int(mx - mw/2)

        #myV = int(my - mh/2)

        w,h,_ = img_maze.shape
        if angle>=-45:
            M = cv2.getRotationMatrix2D((mx,my),(angle),1)
        if angle<-45:
            M = cv2.getRotationMatrix2D((mx,my),(90+angle),1)
        print'angle = ',angle

        #thresh = img_maze

        cv2.imwrite('%s/catkin_ws/src/hector/hector_ika/maps/bedore_rotation.png'%os.path.expanduser('~'),img_maze)
        thresh = cv2.warpAffine(img_maze,M,(h,w))

        cv2.imwrite('%s/catkin_ws/src/hector/hector_ika/maps/after_rotation.png'%os.path.expanduser('~'),thresh)
        ###########################3
        #Döndürme işi biter
        #Bu bölümde görüntüdeki kırmızı ve yeşil bölgelerin piksel koordinatları toplanıp piksel sayılarına bölünerek
        #kırmızı ve yeşil bölgenin orta noktalarının piksel koordinatları bulunur.
        #Ardından bu değerlerin görüntünün sol alt köşesi referans alınarak koordinat değerleri tekrar şekillendirilir
        #(map_server görüntünün sol alt köşesinin orijin kabul ettiği içi)
        #metre/piksel sabiti ile çarpılarak metre cinsinden koordinatları bulunur
        bluePoint = 0
        bluePointX = 0
        bluePointY = 0
        self.height, self.width,_ = thresh.shape
        for x in range(0,self.width):
            for y in range(0,self.height):
                px = thresh[y, x]
                if 60<=int(px[0]) and (int(px[0]) - int(px[2])) >= 20 and (int(px[0]) - int(px[1])) >= 20:
                    thresh[y,x] = 255,255,255
                    #print px
                    bluePoint+=1
                    bluePointX = bluePointX + x
                    bluePointY = bluePointY + y

        bluePointX = bluePointX / bluePoint
        bluePointY = bluePointY / bluePoint

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
            print'diagonal',diagonal
            
            print 'tangent',tangent
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
                denom = 1
            tangent = -1.0*(self.width/2 - bluePointX)/ denom
            
            diagonal = int(math.sqrt(2*0.10/self.resolution*0.10/self.resolution))
            print'diagonal',diagonal
            
            print 'tangent',tangent
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
        
        alfa = math.atan(self.resolution*distance_b/(self.iha_height - 0.4)) 
        
        projection_b = math.tan(alfa)*0.4
        
        bluePointX = bluePointX + k*abs(projection_b*math.sin(math.asin((wall_x_b - self.width/2)/distance_b)))
        bluePointY = bluePointY + l*abs(projection_b*math.cos(math.acos((wall_y_b - self.height/2)/distance_b)))
        
        print 'projection_x',abs(projection_b*math.sin(math.asin((wall_x_b - self.width/2)/distance_b)))
        print 'projection_y',abs(projection_b*math.cos(math.acos((wall_y_b - self.height/2)/distance_b)))
        print'projection_b',projection_b
        
        print 'wall_y_b',wall_y_b
        print 'wall_x_b',wall_x_b
        
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
            print'diagonal',diagonal
            
            print 'tangent',tangent
            counter = 0   
            for y in range(0,self.height/2):
                counter +=1
                if greenPointY < self.height/2:#Burada eğer yeşil noktanın koordinatı merkezden yukarı ise işareti değiştirilir
                    y = -1*y   
                pxX = greenPointX + int(y*1.0/tangent)    
                pxY = greenPointY - y
                px1 = thresh[pxY,pxX]
                if px1[0]<50 and px1[1]<50 and px1[2]<50 and counter > diagonal :#px[0],px[1],px[2] = BGR B,G,R Değerlerinden herhangi biri 30dan düşükse o bölge beyaz değil demektir ve o noktaya gelene kadar
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

	if(abs(greenPointY - self.height/2) <= abs(greenPointX - self.width/2)):
            denom = (self.height/2 - greenPointY)
            if denom == 0:
                denom = 1
            tangent = -1.0*(self.width/2 - greenPointX)/ denom
            
            diagonal = int(math.sqrt(2*0.10/self.resolution*0.10/self.resolution))
            print'diagonal',diagonal
            
            print 'tangent',tangent
            counter = 0

            for x in range(0,self.width/2):
                counter +=1
                if greenPointX < self.width/2:#Burada eğer yeşil noktanın koordinatı merkezden yukarı ise işareti değiştirilir
                    x = -1*x     
                pxX = greenPointX - x
                pxY = greenPointY + int(x*1.0/tangent)
                px1 = thresh[pxY,pxX ]
                if px1[0]<50 and px1[1]<50 and px1[2]<50 and counter > diagonal :#px[0],px[1],px[2] = BGR B,G,R Değerlerinden herhangi biri 30dan düşükse o bölge beyaz değil demektir ve o noktaya gelene kadar
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
        
        print 'projection_x',abs(projection*math.sin(math.asin((wall_x - self.width/2)/distance)))
        print 'projection_y',abs(projection*math.cos(math.acos((wall_y - self.height/2)/distance)))
        print'projection',projection
        
        print 'wall_y',wall_y
        print 'wall_x',wall_x
        
        LBGX = greenPointX - 0  + k*abs(projection*math.sin(math.asin((wall_x - self.width/2)/distance)))/self.resolution    # 'k' ve 'l' değerleri yeşil noktanın sol alt köşeye göre koordinatlarını ayarlamak için
        
        LBGY = -greenPointY - (-1*self.height) + l*abs(projection*math.cos(math.acos((wall_y - self.height/2)/distance)))/self.resolution
        
        # Sol üst köşeyi orijin olarak aldığı için bütün 'y' değerleri bunun altında kalacak ve negatif olacaktır dolayısıyla '-1' ile çarpılırlar   
        
        self.green_originX = -LBGX
        self.green_originY = -LBGY
 	#Duvar Yüksekliğinden kaynaklı kırmızı noktanın mesafe kaybını giderme (duvarın izdüşümünü hesaplama)
	######################################################           
	if(abs(redPointY - self.height/2) > abs(redPointX - self.width/2)):#Eğer kırmızı noktanın fotoğrafın orta noktasına olan 'y' uzaklığı 'x' uzaklığından büyükse eğim değeri 'y'ye göre hesaplanır. Aynı şekilde diğer durumda 'x'e göre bunun sebebei pikseller üzerinde teker teker ilerlemeyi sağlamaktır
            tangent_r = -1*(self.height/2 - redPointY)/(self.width/2 - redPointX) #Tangent between center point and redpoint
            
            diagonal = int(math.sqrt(2*0.10/self.resolution*0.10/self.resolution))
            print'diagonal',diagonal
            
            print 'tangent_r',tangent_r
            counter = 0

            for y in range(0,self.height/2):
                counter +=1
                if redPointY < self.height/2:#Burada eğer yeşil noktanın koordinatı merkezden yukarı ise işareti değiştirilir
                    y = -1*y
                pxX = redPointX + int(y*1.0/tangent_r)
                pxY = redPointY - y
                px1 = thresh[pxY,pxX]
                if px1[0]<50 and px1[1]<50 and px1[2]<50 and counter > diagonal :#px[0],px[1],px[2] = BGR B,G,R Değerlerinden herhangi biri 30dan düşükse o bölge beyaz değil demektir ve o noktaya gelene kadar
                    #px1[0],px1[1],px1[2] = 0,0,250                                 #Robotun rengi koyu olduğu için duvara gelmeden robotun üzerinde durabilir o yüzden eğer piksel sayısı robotun köşegeninin yarısından fazla olmalı
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

	if(abs(redPointY - self.height/2) <= abs(redPointX - self.width/2)):#
            tangent_r = -1*(self.width/2 - redPointX)/(self.height/2 - redPointY)
            
            diagonal = int(math.sqrt(2*0.10/self.resolution*0.10/self.resolution))
            print'diagonal',diagonal
            
            print 'tangent',tangent_r
            counter = 0
            for x in range(0,self.width/2):
                counter +=1
                if redPointX < self.width/2:#Burada eğer yeşil noktanın koordinatı merkezden yukarı ise işareti değiştirilir
                    x = -1*x  
                pxX_r = redPointX - x
                pxY_r = redPointY + int(x*1.0/tangent_r)
                px1 = thresh[pxY_r,pxX_r ]
                if px1[0]<50 and px1[1]<50 and px1[2]<50 and counter > diagonal :#px[0],px[1],px[2] = BGR B,G,R Değerlerinden herhangi biri 30dan düşükse o bölge beyaz değil demektir ve o noktaya gelene kadar
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
        
        print 'projection_x_r',abs(projection_r*math.sin(math.asin((wall_x_r - self.width/2)/distance_r)))
        print 'projection_y_r',abs(projection_r*math.cos(math.acos((wall_y_r - self.height/2)/distance_r)))
        print'projection',projection_r
        
        print 'wall_y_r',wall_y_r
        print 'wall_x_r',wall_x_r
        #Bu bölümde bulunan izdüşüm(projection) değeri x ve y bileşenlerine ayrılarak bu değerler kırmızı noktanın koordinatlarına eklenir
        #Ardından kırmızı noktanın koordinatları sol alt köşe referans alınarak yeniden belirlenir
        LBRX = redPointX - 0  + k*abs(projection_r*math.sin(math.asin((wall_x_r - self.width/2)/distance_r)))/self.resolution    
        
        LBRY = -redPointY - (-1*self.height) + l*abs(projection_r*math.cos(math.acos((wall_y_r - self.height/2)/distance_r)))/self.resolution # Sol üst köşeyi orijin olarak aldığı için bütün 'y' değerleri bunun altında kalacak ve negatif olacaktır
	
        print 'self.width',self.width
        print 'self.height',self.height
        self.RVIZXR = LBRX - LBGX  # yeşil nokta RVIZ'de(map_server) orijin (0,0) olarak alınacağı için kırmızı noktanın 'x' ve 'y' değerleri bu nokta referans alınarak tekrar düzenlenir
        self.RVIZYR = LBRY - LBGY
        self.RVIZXB = -LBBX
        self.RVIZYB = -LBBY
        
        self.RVIZXG = LBGX-LBBX  #Yeşil noktanın koordinatları kadar harita ters tarafa hareket ettirilir böylelikle yeşil nokta (ika) tam (0,0) noktasında oluyoruz
        self.RVIZYG = LBGY-LBBY
        print'self.RVIZXG',self.RVIZXG
        print'self.RVIZYG',self.RVIZYG
        print'self.RVIZXR',self.RVIZXR
        print'self.RVIZYR',self.RVIZYR
        cv2.imwrite('%s/catkin_ws/src/hector/hector_ika/maps/map1.png'%os.path.expanduser('~'),thresh)
        print 'thresh'
        #cv2.circle(thresh,(redPointX,redPointY), 4, (255,40,153), -1)
        #cv2.circle(thresh,(greenPointX,greenPointY),4, (255,40,153), -1)
        if self.map_flag == 2:
            cv2.drawContours(thresh,[box],0,(0,0,0),int(self.wall_thickness/self.resolution)) # Haritayı ikini kez oluştururken 
        cv2.imwrite('%s/catkin_ws/src/hector/hector_ika/maps/map.png'%os.path.expanduser('~'),thresh)

        return thresh

    def map_creator(self, thresh): #Bu fonksiyon kırpılmış resmi alıp bir .pgm ve .yaml (Map) oluşturuyor.
        current_ugv_yaw = self.ugv_theta
        print 'iha_height',self.iha_height
        image = "map.pgm"
        name = "map.yaml"
        #ika ve labirentin duvarı arasındaki yeşil bölgeyi beyaza boyayarak haritalama esnasında duvarla robotun birbirine kaynaşmasını engelliyor
	greenPoint = 0
        greenPointX = 0
        greenPointY = 0
        for x in range(0,self.width):
            for y in range(0,self.height):
                px = thresh[y, x]
                if 60<=int(px[1]) and (px[1]) - int(px[0]) >= 20 and  (int(px[1]) - int(px[2])) >= 20:
                    thresh[y,x] = 255,255,255
                    greenPoint+=1
                    greenPointX = greenPointX + x
                    greenPointY = greenPointY + y
        gray = cv2.cvtColor(thresh, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)
        _, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

        min_length = self.width
        if(min_length>self.height):        #En ve boydan hangisinin piksel değeri daha küçükse o piksel hesaplama için kullanılır
            min_length = self.height

        print min_length
         #hfov değeri olan 130'un yarısı olan 65(1.134464014 radyan) derecenin tanjantı ile
        print 'self.dim1[0]',self.dim1[0]                            #ihanın o anki yüksekli değerinin çarpımının iki katı bize kameranın bir ucundan diğerine olan
                                                                     #metre cinsinden uzaklığı verir. Bunu resmin(640x480) genişlik değerine yani 640'a bölerek
                                                                     #çözünürlük değeri bulunur
       
        print 'self.width',self.width
        #Gürültü temizleme bölümü
        length = []#Kontür uzunluklarını depolamak için bir array oluşturulur
        for i in range(len(contours)):#
                length.insert(i,cv2.arcLength(contours[i],True))

        for k in range(len(contours)):
                if length[k]*self.resolution <=1.8:#Kontürün uzunluk değerini çözünürlük değeri ile çarparak metre cinsinden uzunlukları bulunur. Etrafını sardıkları nesnenin boyu 1.8 metreden küçük ise bself.unu beyaza boyar.
                        cv2.drawContours(thresh, contours, k, (111,111,111),-1)#Kontür kalınlığının negatif değer almasının sebebi nesnenin etrafını sarmak yerine onu boyamasını sağlamak
        cv2.imwrite('%s/catkin_ws/src/hector/hector_ika/maps/thresh.png'%os.path.expanduser('~'),thresh)#Pgm dosyası kayıt edilir
        ret, thresh = cv2.threshold(thresh, 240, 255, cv2.THRESH_BINARY_INV)# Ve griye boyanmış gürültü nesneleri silinir

        cv2.imwrite('%s/catkin_ws/src/hector/hector_ika/maps/map.pgm'%os.path.expanduser('~'),thresh)#Pgm dosyası kayıt edilir

        f = open('%s/catkin_ws/src/hector/hector_ika/maps/%s' %(os.path.expanduser('~'),name),"w+")#Yaml dosyası oluşturulur
        f.write("image: %s\r\n" % image)
        f.write("resolution: %.10f\r\n" % self.resolution)
        if self.map_flag == 1:
            f.write("origin: ['%.10f', '%.10f', %.10f]\r\n"% (self.RVIZXB*self.resolution  , self.RVIZYB*self.resolution,0.0000))
            
        if self.map_flag == 2:
            f.write("origin: ['%.10f', '%.10f', 0.000000]\r\n"% (self.green_originX*self.resolution  , self.green_originY*self.resolution))
            
        f.write("negate: 0\r\n") # for invers thresh negate: 1
        f.write("occupied_thresh: 0.8\r\n")
        f.write("free_thresh: 0.215686275\r\n")
        f.close()
        quat = tf.transformations.quaternion_from_euler(0, 0, self.init_ugvTheta + 1.570796327)
        

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
    Goal_Height = 6.3
    time.sleep(2)
    rospy.init_node("uav_control_node", anonymous=True)
    robot = RobotManager()
    robot.rate.sleep()
    #robot.ugv_mover()
    while True:
        robot.hector_mover(Goal_Height)#Goal height is 6 meters
        #robot.start_location_adjuster()
        #cv2.imshow('uav_loc_cam',robot.img_loc)
        cv2.imshow('robot.img_cam',robot.img_cam)
        cv2.imshow('downcam_raw',robot.downcam_image_data)
        #cv2.imshow('self.resized_downcam',robot.resized_downcam)
        
        cv2.waitKey(100)
        instant_resolution = 2.0*robot.uav_odom[2]*math.tan(1.069886831)/(robot.width_ratio*robot.undistortion_coefficient)
        if robot.is_there_red_and_green() is True and (Goal_Height - 0.3) <= robot.uav_odom[2] and robot.uav_odom[2] <= (Goal_Height + 0.3) and abs(robot.t - robot.b)*math.pow(instant_resolution,2) < 0.5 and abs(robot.r - robot.l)*math.pow(instant_resolution,2)<0.5:# Hem yeşili hem kırmızıyı ayını anda göüyorsa, yüksekliği hedeflenen yükseklikten büyükse ve üst, alt, sağ ve sol alanların aralarındaki farklar 0.5m^2'den küçükse fotoğrafı çek
            cv2.destroyAllWindows()
            photograph = robot.undistorted_img
            photographing_height = robot.uav_odom[2]
            print'photographing_height',photographing_height
            print('Cekiyorum')
            robot.map_creator(robot.image_croper(photographing_height,photograph))
            #subprocess.Popen("roslaunch ika_navigation amcl_demo.launch map_file:=/home/$(whoami)/catkin_ws/src/hector/hector_ika/maps/map.yaml")
            os.system("bash /home/said/catkin_ws/src/hector/hector_ika/src/Secondary_Map.sh")
            time.sleep(20)
            print'Map is created'
            break
    cv2.destroyAllWindows()
    robot.move_base.send_goal(robot.goal)

    #--     İKA'nın kırmızı bitiş yazısını gördüğündeki davranışı
    
    while robot.move_base_status != "Goal reached." or robot.move_base_status == None:
        robot.rate.sleep()

    robot.ugv_yaw_equalizer()

    robot.map_flag = 2
    robot.map_creator(robot.image_croper(photographing_height,photograph))
    #subprocess.Popen("roslaunch ika_navigation amcl_demo.launch map_file:=/home/$(whoami)/catkin_ws/src/hector/hector_ika/maps/map.yaml")
    os.system("bash /home/said/catkin_ws/src/hector/hector_ika/src/Secondary_Map.sh")#Yeni harita için tekrar map_server'i başlatır
    time.sleep(20)    
    
    robot.move_base.send_goal(robot.goal)
    print("Mission Completed!")
    
    while True:
        print 'horororor'
        robot.rate.sleep()
        

if __name__ == "__main__":
    main()
