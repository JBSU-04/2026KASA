cmake_minimum_required(VERSION 3.0.2)
project(planning_mux)
find_package(catkin REQUIRED COMPONENTS rospy nav_msgs mission_supervisor)
catkin_package(CATKIN_DEPENDS rospy nav_msgs mission_supervisor)
catkin_install_python(PROGRAMS scripts/local_path_mux.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
