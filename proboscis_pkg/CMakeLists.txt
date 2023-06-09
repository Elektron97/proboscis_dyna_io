cmake_minimum_required(VERSION 3.0.2)
project(proboscis_pkg)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  dynamixel_sdk
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Dynamixel Library
add_library(dynamixel_utils src/dynamixel_utils.cpp)
target_link_libraries(dynamixel_utils ${catkin_LIBRARIES})

# ROS Library - To Debug
add_library(ros_utils src/ros_utils.cpp)
target_link_libraries(ros_utils ${catkin_LIBRARIES} dynamixel_utils)

add_executable(motor_io src/motor_io.cpp)
target_link_libraries(motor_io ${catkin_LIBRARIES} ros_utils)