cmake_minimum_required(VERSION 3.0.2)
project(mission_pkg)

find_package(catkin REQUIRED COMPONENTS morai_msgs non_gps_msgs roscpp std_msgs)
catkin_package(CATKIN_DEPENDS morai_msgs non_gps_msgs roscpp std_msgs)
include_directories(${catkin_INCLUDE_DIRS})
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(static_obstacle_mission_node src/static_obstacle_mission_node.cpp)
add_dependencies(static_obstacle_mission_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(static_obstacle_mission_node ${catkin_LIBRARIES})

add_executable(dynamic_obstacle_mission_node src/dynamic_obstacle_mission_node.cpp)
add_dependencies(dynamic_obstacle_mission_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(dynamic_obstacle_mission_node ${catkin_LIBRARIES})

add_executable(lane_obstacle_mission_node src/lane_obstacle_mission_node.cpp)
add_dependencies(lane_obstacle_mission_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(lane_obstacle_mission_node ${catkin_LIBRARIES})
