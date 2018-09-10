#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import math


cmdvel = rospy.Publisher("/seyyah/cmd_vel", Twist, queue_size=1)

def callback(data):
    global cmdvel
    
    hrz = data.axes[1]
    vrt = data.axes[3]
    movement_msg = Twist()
    movement_msg.linear.x = hrz*10
    movement_msg.angular.z = vrt*3
    cmdvel.publish(movement_msg)
    
def main():
    
    rospy.Subscriber("/joy", Joy, callback)
    rospy.init_node("Seyyah_joystick", anonymous=True)
    rate = rospy.Rate(10) 

    rate.sleep()
    rospy.spin()                                                                                                                    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
