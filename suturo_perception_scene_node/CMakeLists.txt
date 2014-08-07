cmake_minimum_required(VERSION 2.8.3)
project(suturo_perception_scene_node)

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
  suturo_perception_match_cuboid
  suturo_perception_segmentation
)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

find_package(Boost REQUIRED COMPONENTS system program_options thread)
include_directories(${Boost_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS cv_bridge euroc_c2_msgs roscpp rospy std_msgs perception_utils suturo_perception_msgs suturo_perception_segmentation
  DEPENDS opencv pcl
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(suturo_perception_scene_node src/main.cpp src/suturo_scene_node.cpp)
target_link_libraries(suturo_perception_scene_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES} 
)
add_dependencies(suturo_perception_scene_node suturo_perception_msgs)

add_executable(scene_client src/scene_client.cpp)
target_link_libraries(scene_client
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)
add_dependencies(scene_client suturo_perception_msgs)
