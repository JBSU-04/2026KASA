cmake_minimum_required(VERSION 3.0.2)
project(total_bringup)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs morai_msgs)

catkin_package()

install(PROGRAMS scripts/mpc_load.sh
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS
  scripts/keyboard_override_node.py
  scripts/teleport_reset.py
  scripts/navigation_3d_visualizer_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
