# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "px4_offboard: 1 messages, 0 services")

set(MSG_I_FLAGS "-Ipx4_offboard:/home/odroid/MAV-Project/px4_ws/src/px4_offboard/msg;-Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(px4_offboard_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/odroid/MAV-Project/px4_ws/src/px4_offboard/msg/JoyCommand.msg" NAME_WE)
add_custom_target(_px4_offboard_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "px4_offboard" "/home/odroid/MAV-Project/px4_ws/src/px4_offboard/msg/JoyCommand.msg" "geometry_msgs/Quaternion:std_msgs/Header:geometry_msgs/Point"
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(px4_offboard
  "/home/odroid/MAV-Project/px4_ws/src/px4_offboard/msg/JoyCommand.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/px4_offboard
)

### Generating Services

### Generating Module File
_generate_module_cpp(px4_offboard
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/px4_offboard
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(px4_offboard_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(px4_offboard_generate_messages px4_offboard_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/odroid/MAV-Project/px4_ws/src/px4_offboard/msg/JoyCommand.msg" NAME_WE)
add_dependencies(px4_offboard_generate_messages_cpp _px4_offboard_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(px4_offboard_gencpp)
add_dependencies(px4_offboard_gencpp px4_offboard_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS px4_offboard_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(px4_offboard
  "/home/odroid/MAV-Project/px4_ws/src/px4_offboard/msg/JoyCommand.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/px4_offboard
)

### Generating Services

### Generating Module File
_generate_module_lisp(px4_offboard
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/px4_offboard
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(px4_offboard_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(px4_offboard_generate_messages px4_offboard_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/odroid/MAV-Project/px4_ws/src/px4_offboard/msg/JoyCommand.msg" NAME_WE)
add_dependencies(px4_offboard_generate_messages_lisp _px4_offboard_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(px4_offboard_genlisp)
add_dependencies(px4_offboard_genlisp px4_offboard_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS px4_offboard_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(px4_offboard
  "/home/odroid/MAV-Project/px4_ws/src/px4_offboard/msg/JoyCommand.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/px4_offboard
)

### Generating Services

### Generating Module File
_generate_module_py(px4_offboard
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/px4_offboard
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(px4_offboard_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(px4_offboard_generate_messages px4_offboard_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/odroid/MAV-Project/px4_ws/src/px4_offboard/msg/JoyCommand.msg" NAME_WE)
add_dependencies(px4_offboard_generate_messages_py _px4_offboard_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(px4_offboard_genpy)
add_dependencies(px4_offboard_genpy px4_offboard_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS px4_offboard_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/px4_offboard)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/px4_offboard
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(px4_offboard_generate_messages_cpp geometry_msgs_generate_messages_cpp)
add_dependencies(px4_offboard_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/px4_offboard)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/px4_offboard
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(px4_offboard_generate_messages_lisp geometry_msgs_generate_messages_lisp)
add_dependencies(px4_offboard_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/px4_offboard)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/px4_offboard\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/px4_offboard
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(px4_offboard_generate_messages_py geometry_msgs_generate_messages_py)
add_dependencies(px4_offboard_generate_messages_py std_msgs_generate_messages_py)
