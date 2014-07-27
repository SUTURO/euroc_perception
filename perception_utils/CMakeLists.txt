cmake_minimum_required(VERSION 2.8.3)
project(perception_utils)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  cv_bridge
  roscpp
  sensor_msgs
  std_msgs
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
find_package(Boost 1.48.0 COMPONENTS thread) 

# include opencv
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES perception_utils
  CATKIN_DEPENDS pcl_conversions pcl_ros cv_bridge roscpp sensor_msgs std_msgs
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

add_library(perception_utils src/point_cloud_operations.cpp src/logger.cpp src/publisher_helper.cpp)
target_link_libraries(perception_utils ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES})      

## Declare a cpp executable
add_executable(region_growing src/region_growing.cpp)
target_link_libraries(region_growing
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
