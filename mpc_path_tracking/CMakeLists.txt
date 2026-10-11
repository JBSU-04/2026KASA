cmake_minimum_required(VERSION 3.0.2)
project(mpc_path_tracking)

add_compile_options(-std=c++11 -D_GLIBCXX_USE_CXX11_ABI=1)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ackermann_msgs
  geometry_msgs
  message_generation
  nav_msgs
  roscpp
  rospy
  tf
  sensor_msgs
  std_msgs
  morai_msgs
  new_lane
  waypoint_maker
  erp_driver
  mission_supervisor
)

find_package(Eigen3 REQUIRED)

set(IPOPT_INCLUDE_DIRS /usr/local/include/coin)
set(IPOPT_LIBRARY_DIRS /usr/local/lib)
set(CPPAD_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../CppAD)
set(CPPAD_LOCAL_BUILD_LIB_DIR ${CPPAD_ROOT_DIR}/build/cppad_lib)
set(IPOPT_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../Ipopt_pkg/Ipopt)
set(IPOPT_LOCAL_INCLUDE_DIRS
  ${IPOPT_ROOT_DIR}/src/Common
  ${IPOPT_ROOT_DIR}/src/Interfaces
  ${IPOPT_ROOT_DIR}/src/LinAlg
  ${IPOPT_ROOT_DIR}/src/Algorithm
  ${IPOPT_ROOT_DIR}/src/Algorithm/LinearSolvers
)
set(IPOPT_LOCAL_LIBRARY_DIR ${IPOPT_ROOT_DIR}/build/src/.libs)
set(IPOPT_HSL_LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../Ipopt_pkg/ThirdParty-HSL/.libs)
set(IPOPT_MUMPS_LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../Ipopt_pkg/ThirdParty-Mumps/.libs)
set(MPC_LOCAL_LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib)

include_directories(
  ./include
  ./include/mpc_path_tracking
  ./include/mpc
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${IPOPT_INCLUDE_DIRS}
  ${IPOPT_LOCAL_INCLUDE_DIRS}
  ${CPPAD_ROOT_DIR}/include
  /usr/local/include
  /usr/include
  /usr/include/cppad
)


link_directories(
  ${MPC_LOCAL_LIBRARY_DIR}
  ${IPOPT_LIBRARY_DIRS}
  ${IPOPT_LOCAL_LIBRARY_DIR}
  ${IPOPT_HSL_LIBRARY_DIR}
  ${IPOPT_MUMPS_LIBRARY_DIR}
  ${CPPAD_LOCAL_BUILD_LIB_DIR}
  /usr/local/lib
  /usr/lib
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mpc_path_tracking mpc
  CATKIN_DEPENDS ackermann_msgs geometry_msgs message_generation message_runtime nav_msgs roscpp rospy sensor_msgs std_msgs morai_msgs new_lane waypoint_maker erp_driver mission_supervisor

)

add_executable(mpc_path_tracking src/main.cpp
src/mpc_path_tracking.cpp
src/mpc.cpp
src/new_process.cpp)

add_executable(control_feedback_monitor src/control_feedback_monitor.cpp)

find_library(
  CPPAD_LIB
  NAMES cppad_lib
  PATHS
    ${CPPAD_LOCAL_BUILD_LIB_DIR}
    /usr/local/lib
    /usr/lib
)

if(NOT CPPAD_LIB)
  message(FATAL_ERROR
    "CPPAD_LIB not found. Looked in ${CPPAD_LOCAL_BUILD_LIB_DIR}, /usr/local/lib, /usr/lib")
endif()

find_library(
  IPOPT_LIB
  NAMES ipopt
  PATHS
    ${IPOPT_LOCAL_LIBRARY_DIR}
    /usr/local/lib
    /usr/lib
)

if(NOT IPOPT_LIB)
  message(FATAL_ERROR
    "IPOPT_LIB not found. Looked in ${IPOPT_LOCAL_LIBRARY_DIR}, /usr/local/lib, /usr/lib")
endif()

find_library(
  COINHSL_LIB
  NAMES coinhsl
  PATHS
    ${IPOPT_HSL_LIBRARY_DIR}
    /usr/local/lib
    /usr/lib
)

find_library(
  COINMUMPS_LIB
  NAMES coinmumps
  PATHS
    ${IPOPT_MUMPS_LIBRARY_DIR}
    /usr/local/lib
    /usr/lib
)

unset(METIS_LIB CACHE)
find_library(
  METIS_LIB
  NAMES metis libmetis.so.5 metis_static
  PATHS
    ${MPC_LOCAL_LIBRARY_DIR}
    /lib/x86_64-linux-gnu
    /usr/lib/x86_64-linux-gnu
    /usr/local/lib
    /usr/lib
)

find_library(
  GFORTRAN_LIB
  NAMES gfortran libgfortran.so.5
  PATHS
    /usr/lib/x86_64-linux-gnu
    /usr/lib/gcc/x86_64-linux-gnu/9
    /usr/local/lib
    /usr/lib
)

find_library(
  QUADMATH_LIB
  NAMES quadmath libquadmath.so.0
  PATHS
    /usr/lib/x86_64-linux-gnu
    /usr/lib/gcc/x86_64-linux-gnu/9
    /usr/local/lib
    /usr/lib
)

if(NOT GFORTRAN_LIB)
  message(FATAL_ERROR "GFORTRAN_LIB not found. Looked in /usr/lib/x86_64-linux-gnu, /usr/lib/gcc/x86_64-linux-gnu/9, /usr/local/lib, /usr/lib")
endif()

if(NOT QUADMATH_LIB)
  message(FATAL_ERROR "QUADMATH_LIB not found. Looked in /usr/lib/x86_64-linux-gnu, /usr/lib/gcc/x86_64-linux-gnu/9, /usr/local/lib, /usr/lib")
endif()

target_link_libraries(
  mpc_path_tracking
  ${catkin_LIBRARIES}
  ${CPPAD_LIB}
  ${IPOPT_LIB}
  ${COINHSL_LIB}
  ${COINMUMPS_LIB}
  lapack
  blas
  ${METIS_LIB}
  ${GFORTRAN_LIB}
  ${QUADMATH_LIB}
  pthread
  m
)

set_target_properties(
  mpc_path_tracking
  PROPERTIES
  LINK_FLAGS "-Wl,--disable-new-dtags -Wl,-rpath,${MPC_LOCAL_LIBRARY_DIR}:${IPOPT_LOCAL_LIBRARY_DIR}:${IPOPT_HSL_LIBRARY_DIR}:${IPOPT_MUMPS_LIBRARY_DIR}:${CPPAD_LOCAL_BUILD_LIB_DIR} -Wl,-rpath-link,${MPC_LOCAL_LIBRARY_DIR} -Wl,-rpath-link,${IPOPT_LOCAL_LIBRARY_DIR} -Wl,-rpath-link,${IPOPT_HSL_LIBRARY_DIR} -Wl,-rpath-link,${IPOPT_MUMPS_LIBRARY_DIR} -Wl,-rpath-link,${CPPAD_LOCAL_BUILD_LIB_DIR}"
)

target_link_libraries(control_feedback_monitor ${catkin_LIBRARIES})
