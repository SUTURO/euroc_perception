cmake_minimum_required(VERSION 2.8.3)
project(perception_utils)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  cv_bridge
  roscpp
  sensor_msgs
  std_msgs
  suturo_perception_msgs
  suturo_msgs
)

# pcl
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
# removed because of: http://www.pcl-users.org/Error-creating-object-Voxelgrid-on-Turtlebot-td4031990.html
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

# boost
set(Boost_USE_STATIC_LIBS OFF) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF) 
find_package(Boost 1.48.0 COMPONENTS thread system) 

# include opencv
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES perception_utils
  CATKIN_DEPENDS pcl_conversions pcl_ros cv_bridge roscpp sensor_msgs std_msgs suturo_perception_msgs suturo_msgs
  DEPENDS opencv pcl
)

###########
## Build ##
###########

include_directories(
	include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

set(perception_utils_files
	src/point_cloud_operations.cpp 
	src/logger.cpp 
	src/publisher_helper.cpp 
	src/get_euroc_task_description.cpp
	include/perception_utils/capability.hpp
	include/perception_utils/pipeline_data.hpp
	include/perception_utils/pipeline_object.hpp
	include/perception_utils/point.hpp
	include/perception_utils/shape.hpp
  include/perception_utils/get_euroc_task_description.h
)

add_library(perception_utils ${perception_utils_files})
target_link_libraries(perception_utils ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES})      
add_dependencies(perception_utils suturo_perception_msgs_generate_messages_cpp suturo_msgs_generate_messages_cpp suturo_perception_msgs_gencpp)

add_library(threadsafe_hull src/threadsafe_hull.cpp)
target_link_libraries(threadsafe_hull ${catkin_LIBRARIES})      

add_executable(get_min_max_3d 
	src/basic_io_pcd.cpp
  src/get_min_max_3d.cpp)
target_link_libraries(get_min_max_3d ${catkin_LIBRARIES})      

## Declare a cpp executable
add_executable(region_growing 
  src/region_growing.cpp
  src/basic_io_pcd.cpp)

add_executable(corner_detector 
  src/corner_detector.cpp
  src/basic_io_pcd.cpp)

add_executable(odom_combiner 
  src/odom_combiner.cpp)

add_dependencies(odom_combiner suturo_perception_msgs_generate_messages_cpp suturo_msgs_generate_messages_cpp suturo_perception_msgs_gencpp)

add_executable(cropbox_cloud 
  src/cropbox_cloud.cpp
  src/basic_io_pcd.cpp)

target_link_libraries(region_growing
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES}
)

target_link_libraries(cropbox_cloud
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES}
)

target_link_libraries(corner_detector
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES}
)

## Declare a cpp executable
add_executable(downsample_cloud
	src/downsample_cloud.cpp
	src/basic_io_pcd.cpp
)
target_link_libraries(downsample_cloud
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(odom_combiner
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
