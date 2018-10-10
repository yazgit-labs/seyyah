#!/bin/bash
 
rosservice call /iha/enable_motors true

xterm -e "rqt_image_view" &
sleep 1
xterm -e "rosrun hector_ika goal_Sender.py" &
sleep 1
xterm -e "rosrun hector_ika uav_control_node.py"&
sleep 80
xterm -e "roslaunch ika_description ika_rviz_nav.launch" &
sleep 2 
xterm -e "roslaunch ika_navigation amcl_demo.launch map_file:=/home/$(whoami)/catkin_ws/src/hector/hector_ika/maps/map.yaml" &


