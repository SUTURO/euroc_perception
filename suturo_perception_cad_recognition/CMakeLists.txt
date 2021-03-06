cmake_minimum_required(VERSION 2.8.3)
project(suturo_perception_cad_recognition)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  pcl_conversions
  roscpp
  perception_utils
  suturo_perception_match_cuboid
  shape_msgs
  moveit_ros_planning_interface
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system program_options thread)
# find_package(PCL 1.6 REQUIRED COMPONENTS)
# include_directories(${PCL_INCLUDE_DIRS})
# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES suturo_perception_icp_fitter model_pose_estimation
  CATKIN_DEPENDS suturo_perception_match_cuboid
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
add_library(suturo_perception_icp_fitter
   src/icp_fitter.cpp
   src/IA_method.cpp
   src/IA_centroid.cpp
   src/IA_minmax.cpp
	 #	 src/model_pose_estimation.cpp
	 #	 src/generate_pc_model.cpp
)

## Declare a cpp executable
add_executable(cad_recognition src/cad_recognition.cpp)
add_executable(convert_ply_to_pcd src/convert_ply_to_pcd.cpp)

add_library(generate_pc_model_lib
  src/generate_pc_model.cpp
)

add_library(model_pose_estimation
  src/model_pose_estimation.cpp
)

#add_executable(mpe_prototype
#  src/mpe_prototype.cpp
#)

## Declare a cpp executable
add_executable(generate_pc_model src/generate_pc_model_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(suturo_perception_cad_recognition_node suturo_perception_cad_recognition_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(suturo_perception_icp_fitter
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
)

target_link_libraries(cad_recognition
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
	 suturo_perception_icp_fitter
)

target_link_libraries(convert_ply_to_pcd
   ${catkin_LIBRARIES}
)


target_link_libraries(generate_pc_model_lib
  ${catkin_LIBRARIES}
)

target_link_libraries(model_pose_estimation
  ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
  generate_pc_model_lib
	suturo_perception_icp_fitter
)

#target_link_libraries(mpe_prototype
#  ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
#  model_pose_estimation
#)

target_link_libraries(generate_pc_model
  ${catkin_LIBRARIES}
  generate_pc_model_lib
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS suturo_perception_cad_recognition suturo_perception_cad_recognition_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test tests/tests.cpp)
if(TARGET ${PROJECT_NAME}-test)
	#target_link_libraries(${PROJECT_NAME}-test suturo_perception_icp_fitter ${catkin_LIBRARIES} ${Boost_LIBRARIES})
   target_link_libraries(${PROJECT_NAME}-test suturo_perception_icp_fitter ${catkin_LIBRARIES} ${Boost_LIBRARIES} model_pose_estimation)
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
