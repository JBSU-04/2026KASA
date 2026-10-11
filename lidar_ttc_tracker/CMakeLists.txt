cmake_minimum_required(VERSION 3.0.2)
project(lidar_ttc_tracker)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  TrackedObject.msg
  TrackedObjectArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs message_runtime
)

catkin_install_python(PROGRAMS
  scripts/lidar_ttc_tracker_node.py
  scripts/object_tracking.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
