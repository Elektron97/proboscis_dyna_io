cmake_minimum_required(VERSION 3.0.2)
project(proboscis_pkg)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
)


catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
