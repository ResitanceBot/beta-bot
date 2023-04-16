cmake_minimum_required(VERSION 3.0.2)
project(beacon_gazebo_sim)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  rospy
  std_msgs geometry_msgs message_generation
)

find_package(gazebo REQUIRED)

#find_package(
#        Boost
#        REQUIRED COMPONENTS math
#)


link_directories( ${GAZEBO_LIBRARY_DIRS})

add_message_files(
  FILES
  ReceiverIn.msg BeaconSimPose.msg ReceiverInSyncPacked.msg
)


## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs geometry_msgs
 )

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES beacon_gazebo_sim
#  CATKIN_DEPENDS gazebo gazebo_ros roscpp rospy std_msgs
#  DEPENDS system_lib
        CATKIN_DEPENDS roscpp gazebo_ros std_msgs geometry_msgs message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${GAZEBO_INCLUDE_DIRS}
)

 catkin_install_python(PROGRAMS
   src/odometry_shifter.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

add_library(receiver_model_plugin SHARED src/receiver_model_plugin.cpp src/rssi_noise.cpp)
target_link_libraries(receiver_model_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(beacon_model_plugin SHARED src/beacon_model_plugin.cpp)
target_link_libraries(beacon_model_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(beacons_sync_world_plugin SHARED src/beacons_sync_world_plugin.cpp)
target_link_libraries(beacons_sync_world_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

catkin_package()