cmake_minimum_required(VERSION 3.0.2)
project(lane_perception_pkg)

find_package(catkin REQUIRED COMPONENTS cv_bridge roscpp rospy sensor_msgs std_msgs morai_msgs)
find_package(OpenCV REQUIRED)

catkin_package()
include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(sliding_window_node src/sliding_window_node.cpp)
add_dependencies(sliding_window_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(sliding_window_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

catkin_install_python(PROGRAMS scripts/twinlitenet_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
