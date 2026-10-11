cmake_minimum_required(VERSION 3.0.2)
project(twinlitenet_morai_ros)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  cv_bridge
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/twinlitenet_morai_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
