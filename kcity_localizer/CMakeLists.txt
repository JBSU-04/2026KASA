cmake_minimum_required(VERSION 3.0.2)
project(kcity_localizer)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  tf
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/localization_validity.py
  scripts/map_localizer.py
  scripts/mgeo_loader.py
  scripts/extract_mgeo.py
  scripts/traffic_light_detector.py
  scripts/traffic_light_dataset_collector.py
  scripts/force_green_traffic_lights.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY data launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  OPTIONAL
)
