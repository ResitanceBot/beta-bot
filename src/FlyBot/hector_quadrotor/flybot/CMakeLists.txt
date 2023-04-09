cmake_minimum_required(VERSION 2.8.3)
project(flybot)

find_package(catkin REQUIRED COMPONENTS roscpp )
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

# add_executable(rssi_to_distance src/rssi_to_distance.cpp)
# target_link_libraries(rssi_to_distance ${catkin_LIBRARIES})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY rviz_cfg DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

