cmake_minimum_required(VERSION 3.0.2)
project(new_lane)

set(CMAKE_BUILD_TYPE Release)
add_compile_options(-O3 -march=native)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_conversions
  pcl_ros
  sensor_msgs
  nav_msgs
  visualization_msgs
  mission_supervisor
  ackermann_msgs
  geometry_msgs
  message_generation
  message_runtime
  laser_geometry
  waypoint_maker
  lidar_ttc_tracker
  tf2_ros
  occupancy_avoidance
)

add_message_files(
   FILES
   lidar_topic_msg.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   ackermann_msgs
   sensor_msgs
   waypoint_maker
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)
catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp rospy std_msgs waypoint_maker message_runtime
)

add_executable(lane_changer_node 
   src/lane_changer.cpp 
   src/cubic_spline_planner.cpp
   src/frenet_optimal_trajectory.cpp
   src/polynomials.cpp
)
add_dependencies(lane_changer_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  waypoint_maker_generate_messages_cpp
)

target_link_libraries(lane_changer_node
   ${catkin_LIBRARIES}
)

add_executable(obstacle_avoidance_node
   src/obstacle_avoidance_node.cpp
   src/cubic_spline_planner.cpp
   src/frenet_optimal_trajectory.cpp
   src/polynomials.cpp
)
add_dependencies(obstacle_avoidance_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  waypoint_maker_generate_messages_cpp
)
target_include_directories(obstacle_avoidance_node PRIVATE ${JSONCPP_INCLUDE_DIRS})
target_link_libraries(obstacle_avoidance_node
   ${catkin_LIBRARIES}
   ${JSONCPP_LIBRARIES}
)

catkin_install_python(PROGRAMS
  scripts/obstacle_stop_node.py
  scripts/reference_corridor_viz.py
  scripts/tunnel_map_filter.py
  scripts/tunnel_occupancy_geometry.py
  scripts/tunnel_map_geometry.py
  scripts/build_tunnel_static_map.py
  scripts/escape_planning.py
  scripts/reverse_escape_node.py
  scripts/tunnel_wall_detector.py
  scripts/far_static_obstacle_detector.py
  scripts/obstacle_info_merger.py
  scripts/obstacle_geometry.py
  scripts/curb_zone_visualizer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Shared detector geometry is imported through the package share path.
install(FILES scripts/m8_filtered_input.py scripts/temporal_components.py scripts/tunnel_native_scene.py scripts/tunnel_map_boundary.py scripts/tunnel_region.py scripts/tunnel_scene_policy.py scripts/m8_object_policy.py scripts/tunnel_occupancy_geometry.py scripts/tunnel_map_geometry.py scripts/escape_planning.py scripts/obstacle_geometry.py DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts)

install(DIRECTORY vendor/autoware_compare_map DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/vendor)

install(DIRECTORY docs DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_library(m8_geometry SHARED src/m8_geometry.cpp)
target_compile_features(m8_geometry PRIVATE cxx_std_14)
install(TARGETS m8_geometry LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(FILES scripts/m8_geometry_native.py DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts)

add_executable(tunnel_map_filter_node src/tunnel_map_filter_node.cpp)
target_compile_features(tunnel_map_filter_node PRIVATE cxx_std_14)
target_include_directories(tunnel_map_filter_node PRIVATE ${JSONCPP_INCLUDE_DIRS})
target_link_libraries(tunnel_map_filter_node ${catkin_LIBRARIES} ${JSONCPP_LIBRARIES})
add_dependencies(tunnel_map_filter_node ${catkin_EXPORTED_TARGETS})
install(TARGETS tunnel_map_filter_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS scripts/export_tunnel_runtime_map.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(tunnel_map_kernel_test test/tunnel_map_kernel_test.cpp)
  if(TARGET tunnel_map_kernel_test)
    target_link_libraries(tunnel_map_kernel_test pcl_kdtree pcl_common)
  endif()
endif()
install(DIRECTORY include/new_lane/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

add_executable(tunnel_prefilter_node src/tunnel_prefilter_node.cpp)
target_compile_features(tunnel_prefilter_node PRIVATE cxx_std_14)
target_include_directories(tunnel_prefilter_node PRIVATE ${JSONCPP_INCLUDE_DIRS})
target_link_libraries(tunnel_prefilter_node ${catkin_LIBRARIES} ${JSONCPP_LIBRARIES})
add_dependencies(tunnel_prefilter_node ${catkin_EXPORTED_TARGETS})
install(TARGETS tunnel_prefilter_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(m8_prefilter_test test/m8_prefilter_test.cpp)
endif()
