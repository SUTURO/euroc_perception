cmake_minimum_required(VERSION 2.8.3)
project(suturo_perception_pipeline)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  euroc_c2_msgs
  roscpp
  rospy
  std_msgs
  pcl_ros
  image_transport
  perception_utils
  suturo_perception_match_cuboid
  suturo_perception_centroid_calc
  suturo_perception_shape_detection
)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

find_package(Boost REQUIRED COMPONENTS system program_options thread)
include_directories(${Boost_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES suturo_perception_pipeline
  CATKIN_DEPENDS cv_bridge euroc_c2_msgs roscpp rospy std_msgs perception_utils suturo_perception_match_cuboid suturo_perception_centroid_calc suturo_perception_shape_detection
  DEPENDS pcl
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(suturo_perception_pipeline_files
  src/pipeline.cpp
  include/suturo_perception_pipeline/pipeline.h
)

add_library(suturo_perception_pipeline ${suturo_perception_pipeline_files})
target_link_libraries(suturo_perception_pipeline ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES}) 
