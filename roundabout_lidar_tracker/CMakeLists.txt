cmake_minimum_required(VERSION 3.0.2)
project(roundabout_lidar_tracker)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  mission_planner_msgs
  mission_supervisor
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  tf2
  visualization_msgs
)
find_package(PCL REQUIRED COMPONENTS common filters search segmentation)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES roundabout_centroid_tracker
  CATKIN_DEPENDS mission_planner_msgs nav_msgs pcl_conversions pcl_ros roscpp
                 sensor_msgs tf2 visualization_msgs
  DEPENDS PCL
)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(roundabout_centroid_tracker
  src/centroid_tracker.cpp
  src/cluster_shape_filter.cpp
)
target_compile_options(roundabout_centroid_tracker PRIVATE -O2)
target_link_libraries(roundabout_centroid_tracker ${catkin_LIBRARIES})

add_executable(roundabout_lidar_tracker_node
  src/roundabout_lidar_tracker_node.cpp
)
add_dependencies(roundabout_lidar_tracker_node ${catkin_EXPORTED_TARGETS})
target_compile_options(roundabout_lidar_tracker_node PRIVATE -O2)
target_link_libraries(roundabout_lidar_tracker_node
  roundabout_centroid_tracker
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_centroid_tracker test/test_centroid_tracker.cpp)
  if(TARGET test_centroid_tracker)
    target_link_libraries(test_centroid_tracker roundabout_centroid_tracker)
  endif()
  catkin_add_gtest(test_cluster_shape_filter test/test_cluster_shape_filter.cpp)
  if(TARGET test_cluster_shape_filter)
    target_link_libraries(test_cluster_shape_filter roundabout_centroid_tracker)
  endif()
endif()

install(TARGETS roundabout_centroid_tracker roundabout_lidar_tracker_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
