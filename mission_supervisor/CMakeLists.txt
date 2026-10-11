cmake_minimum_required(VERSION 3.0.2)
project(mission_supervisor)

find_package(catkin REQUIRED COMPONENTS
  rospy std_msgs nav_msgs morai_msgs waypoint_maker
  mission_planner_msgs non_gps_msgs message_generation
)

add_message_files(FILES SectionState.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS rospy std_msgs nav_msgs morai_msgs
               waypoint_maker mission_planner_msgs non_gps_msgs message_runtime)

catkin_install_python(PROGRAMS
  scripts/section_router.py
  scripts/constraint_to_speed_cap.py
  scripts/sequence_monitor.py
  scripts/mission_command_adapter.py
  scripts/mission_debug.py
  scripts/traffic_light_mission.py
  scripts/obstacle_stop_adapter.py
  scripts/roundabout_launch_assist.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
