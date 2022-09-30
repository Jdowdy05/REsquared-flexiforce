cmake_minimum_required(VERSION 2.8.3)
project(arna_example)

## Compile as C++11, supported in ROS Kinetic and newer
## HEBI's code requires that we use this option.
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  hebiros
  rospy
  geometry_msgs
  tf
  tf_conversions
  std_srvs
  actionlib
  eigen_conversions
  kdl_parser 
  sensor_msgs
  cmake_modules
  realtime_tools
  hardware_interface
  controller_interface
  rosbag 
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


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
#  INCLUDE_DIRS include
#  LIBRARIES arna_example
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(
  arna_position_example_node
  src/arna_position_example_node.cpp )

add_executable(
  arna_effort_example_node
  src/arna_effort_example_node.cpp )

add_executable(
  arna_position_node
  src/arna_position_node.cpp )

add_executable(
  flexiforce_position_node
  src/flexiforce_position_node.cpp )

add_executable(
  joint_state_node
  src/joint_state_pub.cpp )

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(
  arna_position_example_node
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(
  arna_effort_example_node
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(
  arna_position_node
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(
  flexiforce_position_node
  ${catkin_EXPORTED_TARGETS}
)
add_dependencies(
   joint_state_node
   ${catkin_EXPORTED_TARGETS}
)
## Specify libraries to link a library or executable target against
target_link_libraries(
  arna_position_example_node
  ${catkin_LIBRARIES}
)

target_link_libraries(
  arna_effort_example_node
  ${catkin_LIBRARIES}
)

target_link_libraries(
  arna_position_node
  ${catkin_LIBRARIES}
)

target_link_libraries(
  flexiforce_position_node
  ${catkin_LIBRARIES}
)

target_link_libraries(
  joint_state_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
 catkin_install_python(PROGRAMS scripts/flexiforce.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/flexiforce_main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_arna_example.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
