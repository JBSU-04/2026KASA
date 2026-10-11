# Modified by KUUVe, 2026.
# Changes: standalone ROS1 catkin build metadata for the vendored Kalman
# filter subset. The Kalman filter implementation sources are unmodified.
cmake_minimum_required(VERSION 3.0.2)
project(amathutils_lib)

add_compile_options(-O2 -Wall -Wextra)
find_package(catkin REQUIRED COMPONENTS roscpp)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES amathutils_lib
  CATKIN_DEPENDS roscpp
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

add_library(amathutils_lib
  src/kalman_filter.cpp
  src/time_delay_kalman_filter.cpp
)
target_link_libraries(amathutils_lib ${catkin_LIBRARIES})

install(TARGETS amathutils_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/amathutils_lib/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
