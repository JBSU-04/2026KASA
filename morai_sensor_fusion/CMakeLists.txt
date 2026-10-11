cmake_minimum_required(VERSION 3.0.2)
project(morai_sensor_fusion)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs morai_msgs nav_msgs rospy sensor_msgs std_msgs tf)

catkin_python_setup()
catkin_package(CATKIN_DEPENDS
  geometry_msgs morai_msgs nav_msgs rospy sensor_msgs std_msgs tf)

catkin_install_python(PROGRAMS
  scripts/morai_measurement_adapter.py
  scripts/ekf_odom_output.py
  scripts/ekf_report_graphs.py
  scripts/tunnel_camera_yaw_detector.py
  scripts/tunnel_ufld_heading.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY config launch models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_measurement.py)
  catkin_add_nosetests(test/test_lane_heading.py)
endif()

install(FILES src/morai_sensor_fusion/ufld_vendor/LICENSE src/morai_sensor_fusion/ufld_vendor/PROVENANCE.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ufld_vendor)
