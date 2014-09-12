cmake_minimum_required(VERSION 2.8.3)
project(suturo_perception_classifier)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_python_setup()

include_directories(
  ${catkin_INCLUDE_DIRS}
)