cmake_minimum_required(VERSION 3.10)
project(tunnel_xicp)
set(CMAKE_CXX_STANDARD 17)
find_package(catkin REQUIRED COMPONENTS roscpp rospy sensor_msgs nav_msgs geometry_msgs std_msgs visualization_msgs tf2_ros message_generation pcl_ros pcl_conversions mission_supervisor)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io kdtree)
find_package(Boost REQUIRED COMPONENTS chrono date_time filesystem program_options system thread timer)
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../.xicp_deps")
find_package(Ceres 2.1 REQUIRED)
add_message_files(FILES XicpStatus.msg)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime roscpp sensor_msgs nav_msgs std_msgs)

# Build the pinned official X-ICP fork, not the ordinary system ICP library.
# Only build its backend; do not bring Open3D SLAM into the vehicle pipeline.
set(PM_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_party/perfectlyconstrained/libpointmatcher")
set(LIBNABO_BUILD_DOXYGEN OFF CACHE BOOL "" FORCE)
set(LIBNABO_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(LIBNABO_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(LIBNABO_BUILD_PYTHON OFF CACHE BOOL "" FORCE)
set(SHARED_LIBS ON CACHE BOOL "" FORCE)
add_subdirectory(third_party/libnabo EXCLUDE_FROM_ALL)
set(MELO_USE_COUT ON)
configure_file(third_party/message_logger/include/message_logger/log/log_messages_backend_config.hpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/generated/message_logger/log/log_messages_backend_config.hpp)
add_library(xicp_logger STATIC
  third_party/message_logger/src/time/Time.cpp
  third_party/message_logger/src/time/TimeStd.cpp
  third_party/message_logger/src/log/log_sentry.cpp)
set_target_properties(xicp_logger PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(xicp_logger PUBLIC third_party/message_logger/include ${CMAKE_CURRENT_BINARY_DIR}/generated)
file(GLOB PM_YAML "${PM_ROOT}/contrib/yaml-cpp-pm/src/*.cpp")
add_library(xicp_yaml STATIC ${PM_YAML})
set_target_properties(xicp_yaml PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(xicp_yaml PUBLIC ${PM_ROOT}/contrib/yaml-cpp-pm/include PRIVATE ${PM_ROOT}/contrib/yaml-cpp-pm/src)
file(GLOB PM_SOURCES "${PM_ROOT}/pointmatcher/*.cpp" "${PM_ROOT}/pointmatcher/ErrorMinimizers/*.cpp" "${PM_ROOT}/pointmatcher/DataPointsFilters/*.cpp")
add_library(xicp_official SHARED ${PM_SOURCES})
configure_file(third_party/qpmad/include/qpmad/config.h.in ${CMAKE_CURRENT_BINARY_DIR}/generated/config.h)
target_include_directories(xicp_official PUBLIC ${PM_ROOT} ${PM_ROOT}/pointmatcher
  ${PM_ROOT}/pointmatcher/DataPointsFilters ${PM_ROOT}/pointmatcher/DataPointsFilters/utils
  third_party/qpmad/include ${CMAKE_CURRENT_BINARY_DIR}/generated ${EIGEN3_INCLUDE_DIRS})
target_compile_definitions(xicp_official PRIVATE NDEBUG MELO_MIN_SEVERITY=3)
target_compile_options(xicp_official PRIVATE -O2 -Wno-deprecated-declarations)
target_link_libraries(xicp_official PUBLIC nabo xicp_yaml xicp_logger Ceres::ceres ${Boost_LIBRARIES})
add_library(tunnel_xicp_backend src/backend.cpp)
target_include_directories(tunnel_xicp_backend PUBLIC include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
target_link_libraries(tunnel_xicp_backend PUBLIC xicp_official ${PCL_LIBRARIES})
add_executable(tunnel_xicp_node src/node.cpp)
add_dependencies(tunnel_xicp_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tunnel_xicp_node tunnel_xicp_backend ${catkin_LIBRARIES})
target_compile_options(tunnel_xicp_backend PRIVATE -O2)
target_compile_options(tunnel_xicp_node PRIVATE -O2)
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_xicp_backend test/test_backend.cpp)
  if(TARGET test_xicp_backend)
    target_link_libraries(test_xicp_backend tunnel_xicp_backend)
    target_compile_options(test_xicp_backend PRIVATE -O2)
    target_compile_definitions(test_xicp_backend PRIVATE XICP_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/xicp.yaml")
  endif()
  catkin_add_nosetests(test/test_map_builder.py)
endif()
catkin_install_python(PROGRAMS scripts/build_wall_map.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS tunnel_xicp_node tunnel_xicp_backend xicp_official xicp_yaml xicp_logger
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY launch config maps DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES scripts/build_wall_map.py DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts)
