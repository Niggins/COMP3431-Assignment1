cmake_minimum_required(VERSION 2.8.3)
project(assign1_2013)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS cv_bridge geometry_msgs nav_msgs roscpp sensor_msgs std_msgs tf image_transport)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

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

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   nav_msgs#   sensor_msgs#   std_msgs
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
  LIBRARIES assign1_2013
  CATKIN_DEPENDS cv_bridge geometry_msgs nav_msgs roscpp sensor_msgs std_msgs tf
  DEPENDS robot_pose_ekf
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(assign1_2013
  src/beacons.cpp
  src/path.cpp
  src/trig.cpp
)

## Declare a cpp executable
add_executable(visual src/nodes/visual.cpp src/path.cpp src/beacons.cpp)
add_executable(simple_nav src/nodes/simple_nav.cpp src/path.cpp)
add_executable(safety_first src/nodes/safety_first.cpp)
add_executable(detector src/nodes/detector.cpp src/beacons.cpp src/trig.cpp)
add_executable(odom_clean src/nodes/odom_clean.cpp)
add_executable(kalman src/nodes/kalman.cpp)
add_executable(distance src/nodes/distance.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(visual ${PROJECT_NAME} roscpp ${PROJECT_NAME}_generate_messages_cpp visual_gencpp)
add_dependencies(simple_nav ${PROJECT_NAME} roscpp ${PROJECT_NAME}_generate_messages_cpp simple_nav_gencpp)
add_dependencies(safety_first ${PROJECT_NAME} roscpp ${PROJECT_NAME}_generate_messages_cpp safety_first_gencpp)
add_dependencies(detector ${PROJECT_NAME} roscpp ${PROJECT_NAME}_generate_messages_cpp safety_first_gencpp)
add_dependencies(odom_clean ${PROJECT_NAME} roscpp ${PROJECT_NAME}_generate_messages_cpp odom_clean_gencpp)
add_dependencies(kalman ${PROJECT_NAME} roscpp ${PROJECT_NAME}_generate_messages_cpp kalman_gencpp)
add_dependencies(distance ${PROJECT_NAME} roscpp ${PROJECT_NAME}_generate_messages_cpp distance_gencpp)

## Specify libraries to link a library or executable target against
target_link_libraries(visual
   ${catkin_LIBRARIES}
)
target_link_libraries(simple_nav
   ${catkin_LIBRARIES}
)
target_link_libraries(safety_first
   ${catkin_LIBRARIES}
)
target_link_libraries(detector
   ${catkin_LIBRARIES}
)
target_link_libraries(odom_clean
   ${catkin_LIBRARIES}
)
target_link_libraries(kalman
   ${catkin_LIBRARIES}
)
target_link_libraries(distance
   ${catkin_LIBRARIES}
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
install(TARGETS assign1_2013 visual simple_nav safety_first detector odom_clean
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_assign1_2013.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
