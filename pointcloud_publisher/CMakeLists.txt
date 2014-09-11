cmake_minimum_required(VERSION 2.8.3)
project(pointcloud_publisher)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  euroc_c2_msgs
  roscpp
  rospy
  std_msgs
  pcl_ros
  image_transport
  perception_utils
)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# enable openmp
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -march=native -mno-avx")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp")
SET(CMAKE_BUILD_TYPE Release)	# Mandatory for openmp!

catkin_package(
  CATKIN_DEPENDS cv_bridge euroc_c2_msgs roscpp rospy std_msgs perception_utils
  DEPENDS opencv pcl
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(PUBLISH_POINTCLOUD_FILES
  src/publish_pointcloud.cpp
  src/projector.cpp
  include/projector.h
)

add_executable(publish_pointcloud ${PUBLISH_POINTCLOUD_FILES})
target_link_libraries(publish_pointcloud
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

