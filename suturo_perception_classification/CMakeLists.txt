cmake_minimum_required(VERSION 2.8.3)
project(suturo_perception_classification)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  shape_msgs
  perception_utils
  suturo_perception_msgs
)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

find_package(Boost REQUIRED COMPONENTS system program_options thread)
include_directories(${Boost_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES suturo_perception_classification
  CATKIN_DEPENDS roscpp std_msgs shape_msgs perception_utils suturo_perception_msgs
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(suturo_perception_classification_files
	include/suturo_perception_classification/task6_classification.hpp
	src/task6_classification.cpp
)

add_library(suturo_perception_classification ${suturo_perception_classification_files})
target_link_libraries(suturo_perception_classification ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES})      
add_dependencies(suturo_perception_classification suturo_msgs_generate_messages_cpp suturo_perception_msgs_generate_messages_cpp suturo_msgs_generate_messages_cpp suturo_perception_msgs_gencpp)
