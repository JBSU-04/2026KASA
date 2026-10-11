cmake_minimum_required(VERSION 3.0.2)
project(lidar_perception_pkg)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs morai_msgs non_gps_msgs pcl_conversions pcl_ros roscpp sensor_msgs std_msgs
  visualization_msgs tf2_ros tf2_geometry_msgs
)

catkin_package(INCLUDE_DIRS include CATKIN_DEPENDS morai_msgs non_gps_msgs roscpp sensor_msgs tf2_ros tf2_geometry_msgs)
include_directories(include ${catkin_INCLUDE_DIRS})
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(clustering_node src/clustering_node.cpp)
add_dependencies(clustering_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(clustering_node ${catkin_LIBRARIES})

add_executable(object_tracking_node src/object_tracking_node.cpp)
add_dependencies(object_tracking_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(object_tracking_node ${catkin_LIBRARIES})
