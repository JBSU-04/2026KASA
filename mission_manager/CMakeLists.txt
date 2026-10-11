cmake_minimum_required(VERSION 3.0.2)
project(mission_manager)

add_compile_options(-std=c++14 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  mission_planner_msgs
  roscpp
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mission_constraint_arbiter
  CATKIN_DEPENDS mission_planner_msgs roscpp std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(mission_constraint_arbiter src/constraint_arbiter.cpp)
add_dependencies(mission_constraint_arbiter ${catkin_EXPORTED_TARGETS})

add_executable(mission_manager_node src/mission_manager_node.cpp)
add_dependencies(mission_manager_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(mission_manager_node
  mission_constraint_arbiter
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_constraint_arbiter test/test_constraint_arbiter.cpp)
  if(TARGET test_constraint_arbiter)
    target_link_libraries(test_constraint_arbiter mission_constraint_arbiter)
  endif()
endif()

install(TARGETS mission_constraint_arbiter mission_manager_node
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
