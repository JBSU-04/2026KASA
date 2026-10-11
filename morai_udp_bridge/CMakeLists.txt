cmake_minimum_required(VERSION 3.0.2)
project(morai_udp_bridge)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  morai_msgs
  rospy
  sensor_msgs
  std_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS diagnostic_msgs morai_msgs rospy sensor_msgs std_msgs
)

catkin_install_python(PROGRAMS
  scripts/apply_morai_udp_config.py
  scripts/morai_udp_bridge_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_apply_config.py)
  catkin_add_nosetests(test/test_protocol.py)
  catkin_add_nosetests(test/test_competition_transport.py)
endif()
