cmake_minimum_required(VERSION 3.0.2)
project(beta_bot_experiments)


find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  beta_bot_localization
  )

 #catkin_package(CATKIN_DEPENDS message_runtime)
 catkin_package(CATKIN_DEPENDS)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(experiments src/experiments.cpp)
add_dependencies(
  experiments ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
  beta_bot_localization_generate_messages_cpp)
target_link_libraries(experiments ${catkin_LIBRARIES})
