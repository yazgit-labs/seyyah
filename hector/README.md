#New 4x4 Skid Steer Robot with Hector

All operations with hector and ika works well for just one type of maze.
You could try it by executing these two commands.

Start simulation

```bash
roslaunch hector_ika hector_ika_maze_1.launch
``` 
Run the script

```bash
bash /home/$(whoami)/catkin_ws/src/hector/hector_ika/src/start.sh
``` 
Or you may follow these steps maually

Launch the ika with a maze
```bash
roslaunch ika_gazebo ika_maze_1.launch 
``` 
Then the navigation package
```bash
roslaunch ika_navigation amcl_demo.launch
``` 
At least the rviz
```bash
roslaunch ika_description ika_rviz_nav.launch
``` 
When you try to solve the maze it's going to do this with the probability of %60
# hector
A mixed package of Hector and Turtlebot3 for Teknofest maze competition

ToDo List
1-Write uav control node
2-Write ugv control node

### Installation
First install Linux, Ros-Kinetic then install Turtlebot3
```bash
sudo apt-get install "ros-kinetic-turtlebot3*"
```
go to a Ros Workspace (like catkin_ws)
```bash
cd ~/catkin_ws/src
```
clone this repo
```bash
git clone https://github.com/bhctsntrk/hector.git
```
move gazebo_models inside to ~/.gazebo/models
```bash
mv ./gazebo_models/models/* ~/.gazebo/models
```
delete empty folder
```bash
rm -rf gazebo_models
```
go back
```bash
cd ..
```
compile hector
```bash
catkin_make
```
### Using and Running
Start simulation
```bash
roslaunch hector_turtlebot3 hector_turtlebot3.launch
```
Run this bash file then everything happens automatically
```bash
bash /home/$(whoami)/catkin_ws/src/hector/hector_turtlebot3/src/start.sh
```
Or you can do everthing separately instead

Start motors
```bash
rosservice call \iha/enable_motors true
```
then you can manually control the vehicles or start AI for vehicles

To control the Turtlebot3 manually
```bash
rosrun hector_turtlebot3 turtlebot3_keyboard_teleop.py
```
To control the Hector manually
```bash
rosrun hector_turtlebot3 hector_keyboard_teleop.py
```
To show the image comes from the bottom camera of the hector
```bash
rqt_image_view
```
Launch subscriber node which send goal which is taken from uav_control_node to move_base
```bash
rosrun hector_turtlebot3 goal_Sender.py
```
Start uav control node AI
```bash
rosrun hector_turtlebot3 uav_control_node.py
```
Launch the turtlebot3 navigation package and rviz(if your turtlebot3 is recently updated)
```bash
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=/home/$(whoami)/catkin_ws/src/hector/hector_turtlebot3/maps/map.yaml
```


