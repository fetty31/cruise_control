cmake_minimum_required(VERSION 3.0.2)
project(cruise_control)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "Release")

message(STATUS "LONGITUDINAL PID says hi :)")

find_package(catkin REQUIRED COMPONENTS
  as_msgs
  roscpp
  rospy
  std_msgs
  ctrl_msgs
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/long.cfg
)

catkin_package(
 INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/main.cpp
                              src/pid.cpp
                              src/longitudinal.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# add_dependencies(${PROJECT_NAME} as_msgs_generate_messages_cpp)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)