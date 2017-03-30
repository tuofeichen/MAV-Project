cmake_minimum_required(VERSION 2.8.3)
project(flow_logger)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
)


set (CMAKE_CXX_FLAGS "-std=c++11")

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES flow_logger
#  CATKIN_DEPENDS mavros px4flow roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  /home/tuofeichen/SLAM/MAV-Project/px4_ws/src/flow_logger/matplotlib-cpp
  ${catkin_INCLUDE_DIRS}
)

add_executable(flow_logger src/logger.cpp)
target_link_libraries(
flow_logger
${catkin_LIBRARIES}
python2.7
)
