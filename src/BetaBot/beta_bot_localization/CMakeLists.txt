cmake_minimum_required(VERSION 3.0.2)
project(beta_bot_localization)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  hector_uav_msgs
  nav_msgs
  roscpp
  sensor_msgs
  tf
  tf2
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${GAZEBO_INCLUDE_DIRS}
)

add_executable(topicAdapter src/topicAdapter/topicAdapter.cpp)
target_link_libraries(topicAdapter ${catkin_LIBRARIES})
add_executable(topicAdapterControls src/topicAdapter/topicAdapterControls.cpp)
target_link_libraries(topicAdapterControls ${catkin_LIBRARIES})