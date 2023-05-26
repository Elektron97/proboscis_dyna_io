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

# This will create executable of the node
add_executable(motor_io src/motor_io.cpp)
# This will link executable to the appropriate libraries
target_link_libraries(motor_io ${catkin_LIBRARIES})