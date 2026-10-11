cmake_minimum_required(VERSION 3.0.2)
project(aeb_safety)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  AebCommand.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs message_runtime
)

catkin_install_python(PROGRAMS
  scripts/aeb_decision_node.py
  scripts/intersection_manager_node.py
  scripts/aeb_arbiter_node.py
  scripts/control_arbiter_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
