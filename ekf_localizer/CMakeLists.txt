# Modified by KUUVe, 2026.
# Changes: standalone ROS1 catkin build metadata and reduced dependencies for
# the vendored EKF subset used by the MORAI integration.
cmake_minimum_required(VERSION 3.0.2)
project(ekf_localizer)

add_compile_options(-O2 -Wall -Wextra -std=c++14)
find_package(catkin REQUIRED COMPONENTS
  amathutils_lib geometry_msgs roscpp sensor_msgs std_msgs tf2 tf2_ros)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS amathutils_lib geometry_msgs roscpp sensor_msgs std_msgs tf2 tf2_ros)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
add_executable(ekf_localizer src/ekf_localizer_node.cpp src/ekf_localizer.cpp)
add_dependencies(ekf_localizer ${catkin_EXPORTED_TARGETS})
target_link_libraries(ekf_localizer ${catkin_LIBRARIES})

install(TARGETS ekf_localizer RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ekf_localizer/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
