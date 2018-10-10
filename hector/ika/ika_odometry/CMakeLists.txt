cmake_minimum_required(VERSION 2.8.3)
project(ika_odometry)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  joy
  sensor_msgs
  roscpp
  rosconsole
  rospy
  tf
  tf2
#  rosserial_arduino
  rosserial_client
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ika_v2
#  CATKIN_DEPENDS geometry_msgs joy roscpp rospy tf tf2 turtlesim
#  DEPENDS system_lib
)

# TODO implement arduiono-cmake here
# rosserial_generate_ros_lib(
# 	PACKAGE rosserial_arduino
# 	SCRIPT make_libraries.py
# )
#
# rosserial_configure_client(
# 	DIRECTORY firmware
# 	TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
# )
#
# rosserial_add_client_target(firmware main_firmware ALL)
# rosserial_add_client_target(firmware main_firmware-upload)

add_executable(ika_odom_publisher src/odom_publisher.cpp)
target_link_libraries(ika_odom_publisher ${catkin_LIBRARIES})
