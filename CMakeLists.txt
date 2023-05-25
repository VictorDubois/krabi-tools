cmake_minimum_required(VERSION 2.8.3)
project(krabi_tools)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  krabi_msgs
)


catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
