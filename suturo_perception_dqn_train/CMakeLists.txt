cmake_minimum_required(VERSION 2.8.3)
project(suturo_perception_dqn_train)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  pcl_conversions
  roscpp
  perception_utils
  shape_msgs
  suturo_msgs
  suturo_perception_cad_recognition
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system program_options thread)
# find_package(PCL 1.6 REQUIRED COMPONENTS)
# include_directories(${PCL_INCLUDE_DIRS})
# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES 
  CATKIN_DEPENDS suturo_perception_match_cuboid suturo_perception_icp_fitter model_pose_estimation
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library

## Declare a cpp executable
add_executable(puzzle_pcd_gen src/puzzle_pcd_gen.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(puzzle_pcd_gen
  ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
  generate_pc_model_lib
	suturo_perception_icp_fitter
)
