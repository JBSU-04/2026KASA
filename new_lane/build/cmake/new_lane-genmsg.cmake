# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "new_lane: 1 messages, 0 services")

set(MSG_I_FLAGS "-Inew_lane:/home/kuuve/catkin_ws/src/new_lane/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/home/kuuve/catkin_ws/src/geometry_msgs/msg;-Iackermann_msgs:/home/kuuve/catkin_ws/src/ackermann_msgs/msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg;-Iwaypoint_maker:/home/kuuve/catkin_ws/src/waypoint_maker/msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(new_lane_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/kuuve/catkin_ws/src/new_lane/msg/lidar_topic_msg.msg" NAME_WE)
add_custom_target(_new_lane_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "new_lane" "/home/kuuve/catkin_ws/src/new_lane/msg/lidar_topic_msg.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(new_lane
  "/home/kuuve/catkin_ws/src/new_lane/msg/lidar_topic_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/new_lane
)

### Generating Services

### Generating Module File
_generate_module_cpp(new_lane
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/new_lane
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(new_lane_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(new_lane_generate_messages new_lane_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kuuve/catkin_ws/src/new_lane/msg/lidar_topic_msg.msg" NAME_WE)
add_dependencies(new_lane_generate_messages_cpp _new_lane_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(new_lane_gencpp)
add_dependencies(new_lane_gencpp new_lane_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS new_lane_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(new_lane
  "/home/kuuve/catkin_ws/src/new_lane/msg/lidar_topic_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/new_lane
)

### Generating Services

### Generating Module File
_generate_module_eus(new_lane
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/new_lane
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(new_lane_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(new_lane_generate_messages new_lane_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kuuve/catkin_ws/src/new_lane/msg/lidar_topic_msg.msg" NAME_WE)
add_dependencies(new_lane_generate_messages_eus _new_lane_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(new_lane_geneus)
add_dependencies(new_lane_geneus new_lane_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS new_lane_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(new_lane
  "/home/kuuve/catkin_ws/src/new_lane/msg/lidar_topic_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/new_lane
)

### Generating Services

### Generating Module File
_generate_module_lisp(new_lane
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/new_lane
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(new_lane_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(new_lane_generate_messages new_lane_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kuuve/catkin_ws/src/new_lane/msg/lidar_topic_msg.msg" NAME_WE)
add_dependencies(new_lane_generate_messages_lisp _new_lane_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(new_lane_genlisp)
add_dependencies(new_lane_genlisp new_lane_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS new_lane_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(new_lane
  "/home/kuuve/catkin_ws/src/new_lane/msg/lidar_topic_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/new_lane
)

### Generating Services

### Generating Module File
_generate_module_nodejs(new_lane
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/new_lane
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(new_lane_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(new_lane_generate_messages new_lane_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kuuve/catkin_ws/src/new_lane/msg/lidar_topic_msg.msg" NAME_WE)
add_dependencies(new_lane_generate_messages_nodejs _new_lane_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(new_lane_gennodejs)
add_dependencies(new_lane_gennodejs new_lane_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS new_lane_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(new_lane
  "/home/kuuve/catkin_ws/src/new_lane/msg/lidar_topic_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/new_lane
)

### Generating Services

### Generating Module File
_generate_module_py(new_lane
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/new_lane
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(new_lane_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(new_lane_generate_messages new_lane_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kuuve/catkin_ws/src/new_lane/msg/lidar_topic_msg.msg" NAME_WE)
add_dependencies(new_lane_generate_messages_py _new_lane_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(new_lane_genpy)
add_dependencies(new_lane_genpy new_lane_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS new_lane_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/new_lane)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/new_lane
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(new_lane_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(new_lane_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET ackermann_msgs_generate_messages_cpp)
  add_dependencies(new_lane_generate_messages_cpp ackermann_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(new_lane_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET waypoint_maker_generate_messages_cpp)
  add_dependencies(new_lane_generate_messages_cpp waypoint_maker_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/new_lane)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/new_lane
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(new_lane_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(new_lane_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET ackermann_msgs_generate_messages_eus)
  add_dependencies(new_lane_generate_messages_eus ackermann_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(new_lane_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET waypoint_maker_generate_messages_eus)
  add_dependencies(new_lane_generate_messages_eus waypoint_maker_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/new_lane)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/new_lane
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(new_lane_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(new_lane_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET ackermann_msgs_generate_messages_lisp)
  add_dependencies(new_lane_generate_messages_lisp ackermann_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(new_lane_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET waypoint_maker_generate_messages_lisp)
  add_dependencies(new_lane_generate_messages_lisp waypoint_maker_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/new_lane)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/new_lane
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(new_lane_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(new_lane_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET ackermann_msgs_generate_messages_nodejs)
  add_dependencies(new_lane_generate_messages_nodejs ackermann_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(new_lane_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET waypoint_maker_generate_messages_nodejs)
  add_dependencies(new_lane_generate_messages_nodejs waypoint_maker_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/new_lane)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/new_lane\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/new_lane
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(new_lane_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(new_lane_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET ackermann_msgs_generate_messages_py)
  add_dependencies(new_lane_generate_messages_py ackermann_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(new_lane_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET waypoint_maker_generate_messages_py)
  add_dependencies(new_lane_generate_messages_py waypoint_maker_generate_messages_py)
endif()
