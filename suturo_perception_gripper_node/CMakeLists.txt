cmake_minimum_required(VERSION 2.8.3)
project(suturo_perception_gripper_node)

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
  suturo_perception_segmentation
  suturo_perception_pipeline
)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

find_package(Boost REQUIRED COMPONENTS system program_options thread)
include_directories(${Boost_INCLUDE_DIRS})

generate_dynamic_reconfigure_options(
  cfg/SuturoPerception.cfg
)

catkin_package(
  CATKIN_DEPENDS cv_bridge euroc_c2_msgs roscpp rospy std_msgs perception_utils suturo_perception_msgs suturo_perception_segmentation suturo_perception_pipeline
  DEPENDS opencv pcl
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(suturo_perception_gripper_node src/main.cpp src/suturo_gripper_node.cpp)
target_link_libraries(suturo_perception_gripper_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES} 
)

add_executable(gripper_client src/gripper_client.cpp)
target_link_libraries(gripper_client
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)