cmake_minimum_required(VERSION 3.0.2)
project(control_pkg)

find_package(catkin REQUIRED COMPONENTS morai_msgs non_gps_msgs roscpp std_msgs)
catkin_package(CATKIN_DEPENDS morai_msgs non_gps_msgs roscpp std_msgs)
include_directories(${catkin_INCLUDE_DIRS})
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(control_mux_node src/control_mux_node.cpp)
add_dependencies(control_mux_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(control_mux_node ${catkin_LIBRARIES})

add_executable(vehicle_control_node src/vehicle_control_node.cpp)
add_dependencies(vehicle_control_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(vehicle_control_node ${catkin_LIBRARIES})
