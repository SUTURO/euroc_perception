cmake_minimum_required(VERSION 2.8.3)
project(table_from_depth_img)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
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
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# enable openmp
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp")
SET(CMAKE_BUILD_TYPE Release)	# Mandatory for openmp!

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES table_from_depth_img
  CATKIN_DEPENDS cv_bridge euroc_c2_msgs roscpp rospy std_msgs perception_utils suturo_perception_msgs
  DEPENDS opencv pcl
)

include_directories(
  src
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(calc_pc_from_img_and_depth src/calc_pc_from_img_and_depth.cpp)
target_link_libraries(calc_pc_from_img_and_depth
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

add_executable(extract_table src/main.cpp src/table_from_depth_img_node.cpp)
target_link_libraries(extract_table
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

add_executable(table_client src/table_client.cpp)
target_link_libraries(table_client
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

