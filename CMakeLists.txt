cmake_minimum_required(VERSION 3.0.2)
project(mocopi_ros)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2
)

catkin_install_python(PROGRAMS
  scripts/mocopi_receiver.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/mocopi_ros
)
