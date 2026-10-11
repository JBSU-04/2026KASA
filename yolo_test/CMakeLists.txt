cmake_minimum_required(VERSION 3.0.2)
project(yolo_test)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
  morai_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  udp_sensor_bridge.py
  obstacle_perception_node.py
  pointpainting_node.py
  pointpainting_node_debug.py
  dual_model_test_node_s.py
  dual_model_detector.py
  camera_speed_kick_node.py
  image_capturer_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY weights/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/weights
)
