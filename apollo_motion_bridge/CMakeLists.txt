cmake_minimum_required(VERSION 3.0.2)
project(apollo_motion_bridge)
find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)
catkin_python_setup()
catkin_package(LIBRARIES apollo_motion_measurement)
add_library(apollo_motion_measurement SHARED
  src/measurement_bridge.cpp
  vendor/modules/perception/lidar_tracking/tracker/measurement/measurement_collection.cc)
target_compile_features(apollo_motion_measurement PRIVATE cxx_std_14)
target_include_directories(apollo_motion_measurement PRIVATE compat vendor ${EIGEN3_INCLUDE_DIRS})
install(TARGETS apollo_motion_measurement LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY vendor DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
