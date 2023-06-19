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

#### Libraries ####
# Dynamixel Library
add_library(dynamixel_utils src/dynamixel/dynamixel_utils.cpp)
target_link_libraries(dynamixel_utils ${catkin_LIBRARIES})

# ROS Library
add_library(ros_utils src/dynamixel/ros_utils.cpp)
target_link_libraries(ros_utils ${catkin_LIBRARIES} dynamixel_utils)

# PID Library
add_library(pid_utils src/control/pid.cpp)
target_link_libraries(pid_utils ${catkin_LIBRARIES})

# Math Library
add_library(math_utility src/math/math_utility.cpp)
target_link_libraries(math_utility ${catkin_LIBRARIES})

# Control Library
add_library(control_utils src/control/control_utils.cpp)
target_link_libraries(control_utils ${catkin_LIBRARIES} math_utility pid_utils)

#### Execs ####
# Hardware IO Node
add_executable(motor_io src/dynamixel/motor_io.cpp)
target_link_libraries(motor_io ${catkin_LIBRARIES} ros_utils)

# Controller Node
add_executable(controller src/control/controller.cpp)
target_link_libraries(controller ${catkin_LIBRARIES} control_utils)