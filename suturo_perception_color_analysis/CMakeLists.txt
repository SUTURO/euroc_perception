cmake_minimum_required(VERSION 2.8.3)
project(suturo_perception_color_analysis)

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
  LIBRARIES suturo_perception_color_analysis
  CATKIN_DEPENDS cv_bridge roscpp
  DEPENDS opencv pcl
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(SUTURO_PERCEPTION_COLOR_ANALYSIS_FILES
  src/color_analysis.cpp
  include/suturo_perception_color_analysis/color_analysis.h
)

add_library(suturo_perception_color_analysis ${SUTURO_PERCEPTION_COLOR_ANALYSIS_FILES})
target_link_libraries(suturo_perception_color_analysis 
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBS} 
)

catkin_add_gtest(${PROJECT_NAME}-test 
  test/test.cpp
)
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} ${catkin_LIBRARIES})
  
  find_package(PCL REQUIRED COMPONENTS)
  find_package(OpenCV REQUIRED )

  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS} ${OpenCV_LIBS})
  add_definitions(${PCL_DEFINITIONS})
endif()

