cmake_minimum_required(VERSION 2.8.3)
project(suturo_perception_centroid_calc)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
	roslib
	perception_utils
)

find_package(PCL REQUIRED COMPONENTS)
include_directories(${PCL_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES suturo_perception_centroid_calc
  CATKIN_DEPENDS pcl_ros roscpp roslib perception_utils
  DEPENDS pcl
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(suturo_perception_centroid_files
  src/centroid_calc.cpp
  include/suturo_perception_centroid_calc/centroid_calc.h
)

add_library(suturo_perception_centroid_calc ${suturo_perception_centroid_files})
target_link_libraries(suturo_perception_centroid_calc ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES}) 
