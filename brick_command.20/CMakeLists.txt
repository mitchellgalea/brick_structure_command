cmake_minimum_required(VERSION 2.8.3)
project(brick_command)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  message_runtime
  roscpp
  std_msgs
  std_srvs
  pcl_ros
  tf
  tf_conversions
  tf2
  tf2_geometry_msgs
  roslib
  cv_bridge
  geometry_msgs
  image_transport
  image_geometry
  mrc_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(PCL 1.7 REQUIRED REQUIRED)
find_package(OpenCV REQUIRED)


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
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
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
add_message_files(
  FILES
  BrickStructureLines.msg

)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  StructurePoseEst.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  mrc_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES brick_structure_command
#  CATKIN_DEPENDS geometry_msgs message_generation message_runtime roscpp std_msgs std_srvs
#  DEPENDS system_lib
        CATKIN_DEPENDS
        mrc_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIR}
  ${Boost_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

## Declare a C++ library
add_library(brick_structure
  src/brick_structure/brick.cpp
  include/brick_structure/brick.h
  src/brick_structure/brick_structure.cpp
  include/brick_structure/brick_structure.h
  src/brick_structure/geometry.cpp
  include/brick_structure/geometry.h
  src/brick_structure/pose_estimation.cpp
  include/brick_structure/pose_estimation.h
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(brick_structure ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide

add_executable(${PROJECT_NAME}_node src/brick_command_node.cpp)
add_executable(brick_param_node src/brick_param_node.cpp)
add_executable(brick_structure_gazebo_launch_gen src/brick_structue_gazebo_launch_gen.cpp)
add_executable(structure_pose_est_node src/structure_pose_est_node.cpp)
add_executable(brick_pcd src/brick_pcd.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(brick_param_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(brick_structure_gazebo_launch_gen ${catkin_EXPORTED_TARGETS} )
add_dependencies(structure_pose_est_node ${catkin_EXPORTED_TARGETS} )
add_dependencies(brick_pcd ${catkin_EXPORTED_TARGETS} )
## Specify libraries to link a library or executable target against

target_link_libraries(brick_structure
    ${catkin_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_node
 ${catkin_LIBRARIES}
 brick_structure
)

target_link_libraries(brick_param_node
 ${catkin_LIBRARIES}
 brick_structure
)
target_link_libraries(brick_structure_gazebo_launch_gen
 ${catkin_LIBRARIES}
 brick_structure
)
target_link_libraries(structure_pose_est_node
 ${catkin_LIBRARIES}
 brick_structure
 ${PCL_LIBRARIES}
 ${EIGEN3_LIBRARIES}
)
target_link_libraries(brick_pcd
        ${catkin_LIBRARIES}
        brick_structure
        ${PCL_LIBRARIES}
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
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
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
catkin_add_gtest(brick_structure-test test/brick_structure_test.cpp)
if(TARGET brick_structure-test)
    target_link_libraries(brick_structure-test ${catkin_LIBRARIES} brick_structure)
    message ( STATUS " catkin_LIBRARIES ${catkin_LIBRARIES}")
endif()
