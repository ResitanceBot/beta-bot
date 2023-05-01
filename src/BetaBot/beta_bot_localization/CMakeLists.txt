cmake_minimum_required(VERSION 3.0.2)
project(beta_bot_localization)

set(CMAKE_BUILD_TYPE Debug)

find_package(
  catkin REQUIRED
  COMPONENTS geometry_msgs
             hector_uav_msgs
             nav_msgs
             roscpp
             sensor_msgs
             tf
             tf2
             std_msgs
             beacons_gazebo
             message_generation)

find_package(Eigen3 REQUIRED)

add_message_files(FILES IniLocalization.msg)

generate_messages(DEPENDENCIES geometry_msgs)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR}
                    ${GAZEBO_INCLUDE_DIRS})

add_executable(topicAdapter src/topicAdapter/topicAdapter.cpp)
target_link_libraries(topicAdapter ${catkin_LIBRARIES})

add_executable(topicAdapterControls src/topicAdapter/topicAdapterControls.cpp)
target_link_libraries(topicAdapterControls ${catkin_LIBRARIES})

add_executable(gn_ini src/ini/Gauss_Newton_Algorithm.cpp)
target_link_libraries(gn_ini ${catkin_LIBRARIES})
add_dependencies(gn_ini beta_bot_localization_generate_messages_cpp)

add_executable(ekf_node src/ekf_node/ros_wrapper.cpp src/ekf_node/ekf.cpp)
add_dependencies(
  ekf_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
  beta_bot_localization_generate_messages_cpp)
target_link_libraries(ekf_node ${catkin_LIBRARIES})
