cmake_minimum_required(VERSION 3.0.2)
project(proboscis_dyna_io)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  dynamixel_sdk
  ros_dynamixel_pkg
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#### Libraries ####
# ROS Library
add_library(ros_utils src/ros_utils.cpp)
target_link_libraries(ros_utils ${catkin_LIBRARIES})

#### Execs ####
# Hardware IO Node
add_executable(motor_io src/motor_io.cpp)
target_link_libraries(motor_io ${catkin_LIBRARIES} ros_utils)