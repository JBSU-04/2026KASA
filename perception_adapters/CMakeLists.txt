cmake_minimum_required(VERSION 3.0.2)
project(perception_adapters)
find_package(catkin REQUIRED COMPONENTS rospy lidar_ttc_tracker
             mission_planner_msgs non_gps_msgs)
catkin_package(CATKIN_DEPENDS rospy lidar_ttc_tracker
               mission_planner_msgs non_gps_msgs)
catkin_install_python(PROGRAMS scripts/tracked_object_adapter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
