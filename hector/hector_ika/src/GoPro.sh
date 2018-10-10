#!/bin/bash
xterm -e "export PATH="/home/said/miniconda3/bin:$PATH"
source activate GOMopencv
python /home/said/catkin_ws/src/hector/hector_ika/src/vava.py" &
sleep 10

xterm -e "python /home/said/catkin_ws/src/hector/hector_ika/src/GoPro_streamer_node.py"
# 
# xterm -e "source activate GOMopencv
# python /home/said/catkin_ws/src/hector/hector_ika/src/lala.py"
