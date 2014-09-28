cmake_minimum_required(VERSION 2.8.3)
project(suturo_perception_height_calculation)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  perception_utils
)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

find_package(Boost 1.48.0 COMPONENTS thread) 
set(Boost_USE_STATIC_LIBS OFF) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF) 


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES suturo_perception_height_calculation
  CATKIN_DEPENDS cv_bridge roscpp perception_utils
  DEPENDS opencv pcl
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(SUTURO_PERCEPTION_HEIGHT_CALCULATION_FILES
  src/height_calculation.cpp
  include/suturo_perception_height_calculation/height_calculation.h
)

add_library(suturo_perception_height_calculation ${SUTURO_PERCEPTION_HEIGHT_CALCULATION_FILES})
target_link_libraries(suturo_perception_height_calculation 
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBS} 
  ${catkin_LIBRARIES} 
)

