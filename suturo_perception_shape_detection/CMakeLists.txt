cmake_minimum_required(VERSION 2.8.3)
project(suturo_perception_shape_detection)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  perception_utils
)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

set(Boost_USE_STATIC_LIBS OFF) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF) 
find_package(Boost 1.48.0 COMPONENTS thread) 
include_directories(${Boost_INCLUDE_DIRS}) 
link_directories(${Boost_LIBRARY_DIRS})                                                                                                                                                                                                                  

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES suturo_perception_shape_detection
  CATKIN_DEPENDS roscpp std_msgs perception_utils
  DEPENDS pcl
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(suturo_perception_shape_detection_files
  src/shape_detector.cpp
  include/suturo_perception_shape_detection/shape_detector.h
)

add_library(suturo_perception_shape_detection ${suturo_perception_shape_detection_files})
target_link_libraries(suturo_perception_shape_detection 
  ${Boost_LIBRARIES} 
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)           