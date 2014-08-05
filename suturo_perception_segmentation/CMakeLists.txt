cmake_minimum_required(VERSION 2.8.3)
project(suturo_perception_segmentation)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  euroc_c2_msgs
  roscpp
  rospy
  std_msgs
  pcl_ros
  image_transport
  perception_utils
  suturo_perception_msgs
)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

# enable openmp
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp")
SET(CMAKE_BUILD_TYPE Release)	# Mandatory for openmp!

catkin_package(
  CATKIN_DEPENDS cv_bridge euroc_c2_msgs roscpp rospy std_msgs perception_utils suturo_perception_msgs
  DEPENDS opencv pcl
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#add_executable(suturo_perception_scene_node src/main.cpp src/suturo_scene_node.cpp)
#target_link_libraries(suturo_perception_scene_node
#  ${catkin_LIBRARIES}
#  ${OpenCV_LIBS}
#  ${PCL_LIBRARIES}
#)
#add_dependencies(suturo_perception_scene_node suturo_perception_msgs)