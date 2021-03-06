cmake_minimum_required(VERSION 2.8.3)
project(slam_test)
add_definitions("-x c++")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  roscpp
  # message_generation
  std_msgs
  geometry_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
# FILES
# #   Message1.msg
# #   Message2.msg
#  )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#    DEPENDENCIES

# #   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rgbd_slam
  CATKIN_DEPENDS
  roscpp
  std_msgs
  # message_runtime
#  DEPENDS system_lib

)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)


set (CMAKE_CXX_FLAGS "-std=c++11 -O3 -Ofast -ftree-vectorize -Wall")

# -ftree-vectorizer-verbose=9

set(CMAKE_CXX_LINK_FLAGS "-L/opt/opencv3/lib -L/usr/lib -L/home/tuofeichen/OpenNI-Linux-x64-2.2/Redist")

configure_file(/home/tuofeichen/g2o/config.h.in /home/tuofeichen/g2o/g2o/config.h)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  /usr/include/pcl-1.7
  /usr/include/vtk-5.8
  /home/tuofeichen/g2o
  /home/tuofeichen/g2o/EXTERNAL/csparse
  /usr/include/eigen3
  /home/tuofeichen/OpenNI-Linux-x64-2.2/Include
  /opt/opencv3/include
)

link_directories(

/opt/opencv3/lib
/usr/lib
/home/tuofeichen/g2o/lib
/home/tuofeichen/OpenNI-Linux-x64-2.2/Redist
/opt/ros/indigo/lib/

)


## Declare a C++ executable
add_executable(slam_test src/main.cpp) # src/RosHandler.cpp)


add_library(
SLAM
src/AnalyticBasedTME.cpp
src/FrameToPcConverter.cpp
src/G2oPoseGraph.cpp
src/Mapping.cpp
src/Frame.cpp
src/RANSACBasedTME.cpp
src/TcpClient.cpp
src/SIFT.cpp
src/PointCloudMap.cpp
src/SURF.cpp
src/ORB.cpp
src/OrbDetSurfDesc.cpp
src/UavComProtocolHandler.cpp
src/AsusProLiveOpenNI2.cpp
# src/RosHandler.cpp
# src/Backend.cpp
# src/TcpServer.cpp
# src/ObjDetection.cpp
src/RGDBSimulator.cpp)

target_link_libraries(SLAM
${catkin_LIBRARIES}
g2o_core
g2o_csparse_extension
g2o_types_slam3d
boost_system
boost_thread
pthread
OpenNI2
pcl_io
pcl_common
pcl_visualization
pcl_filters
opencv_core
opencv_xfeatures2d
opencv_features2d
opencv_calib3d
opencv_flann
opencv_highgui
opencv_videoio
opencv_imgproc
opencv_imgcodecs

)

target_link_libraries(slam_test
${catkin_LIBRARIES}
jpeg
png
SLAM
)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(slam_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


## Specify libraries to link a library or executable target against
# target_link_libraries(rgbd_slam_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS rgbd_slam rgbd_slam_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rgbd_slam.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
