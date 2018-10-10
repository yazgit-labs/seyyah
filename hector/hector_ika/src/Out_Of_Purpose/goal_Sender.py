#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Point
import rospy, actionlib, tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class Sending_Goal:
    def __init__(self):
        self.callback_flag = 0
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
        rospy.Subscriber("/Goal_Share",Point,self.goal_callback)
        rospy.spin()
        
    def goal_callback(self,msg):
        self.pose = msg
        self.callback_flag = 1
        print "Goal Coodinates : ",msg.x,msg.y,msg.z
        if(self.callback_flag == 1):
            self.move_base.wait_for_server()
            self.publishgoal(self.pose)# Kontrol nodunda oluşturduğumuz "goal" değişkeninin değerleri move_base'e gönderilir
        
    def publishgoal(self,pose):
	s = str.split(",")
	x = pose.x
	y = pose.y
	th = 0.785398163
	
	goal = MoveBaseGoal()
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.header.frame_id = "map"
	
	goal.target_pose.pose.position.x = x
	goal.target_pose.pose.position.y = y
	goal.target_pose.pose.position.z = 0.0
	
	quat = tf.transformations.quaternion_from_euler(0, 0, th)
	goal.target_pose.pose.orientation.x = quat[0]
	goal.target_pose.pose.orientation.y = quat[1]
	goal.target_pose.pose.orientation.z = quat[2]
	goal.target_pose.pose.orientation.w = quat[3]

	self.move_base.send_goal(goal)
        
def main():
    rospy.init_node("Goal_Sender",anonymous = True)
    goal = Sending_Goal()
    
    #goal.publishgoal(goal.pose)
        
if __name__ == "__main__":
    main()
