cmake_minimum_required(VERSION 2.8.3)
project(table_from_depth_img)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  euroc_c2_msgs
  roscpp
  rospy
  std_msgs
  pcl_ros
  image_transport
  perception_utils
  suturo_perception_msgs
)

find_package(Boost REQUIRED COMPONENTS system program_options thread)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
# removed because of: http://www.pcl-users.org/Error-creating-object-Voxelgrid-on-Turtlebot-td4031990.html
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

# enable openmp
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp")
SET(CMAKE_BUILD_TYPE Release)	# Mandatory for openmp!

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES table_from_depth_img
  CATKIN_DEPENDS cv_bridge euroc_c2_msgs roscpp rospy std_msgs perception_utils suturo_perception_msgs
  DEPENDS opencv pcl
)

include_directories(
  src
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(calc_pc_from_img_and_depth src/calc_pc_from_img_and_depth.cpp)
target_link_libraries(calc_pc_from_img_and_depth
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

add_executable(calc_pc_from_img_and_depth_annex_approved src/calc_pc_from_img_and_depth_annex_approved.cpp)
target_link_libraries(calc_pc_from_img_and_depth_annex_approved
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

add_executable(extract_table src/main.cpp src/table_from_depth_img_node.cpp)
target_link_libraries(extract_table
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES} 
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)
add_dependencies(extract_table suturo_perception_msgs)

add_executable(table_client src/table_client.cpp)
target_link_libraries(table_client
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)
add_dependencies(table_client suturo_perception_msgs)

