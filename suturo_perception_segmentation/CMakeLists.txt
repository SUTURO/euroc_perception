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
)
# enable openmp
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -march=native -mno-avx")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp")
SET(CMAKE_BUILD_TYPE Release)	# Mandatory for openmp!

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES suturo_perception_segmentation
  CATKIN_DEPENDS cv_bridge euroc_c2_msgs roscpp rospy std_msgs perception_utils 
  DEPENDS pcl
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(suturo_perception_segmentation_files
  src/segmenter.cpp
  src/projection_segmenter.cpp
  src/task6_segmenter.cpp
  src/task4_segmenter.cpp
  include/suturo_perception_segmentation/segmenter.h
  include/suturo_perception_segmentation/projection_segmenter.h
  include/suturo_perception_segmentation/task6_segmenter.h
  include/suturo_perception_segmentation/task4_segmenter.h
)

add_library(suturo_perception_segmentation ${suturo_perception_segmentation_files})
target_link_libraries(suturo_perception_segmentation ${PCL_LIBRARIES} ${catkin_LIBRARIES}) 
