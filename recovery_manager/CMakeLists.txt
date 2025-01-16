cmake_minimum_required(VERSION 3.0.2)
project(recovery_manager)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  costmap_2d
  dynamic_reconfigure
  geometry_msgs
  message_generation
  move_base_msgs
  nav_msgs
  roscpp
  rospy
  lexxauto_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES safety_direction_recovery
  CATKIN_DEPENDS
    dynamic_reconfigure
    geometry_msgs
    move_base_msgs
    nav_msgs
    roscpp
    lexxauto_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(switch_params_manager STATIC src/switch_params_manager.cpp)
add_dependencies(switch_params_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(switch_params_manager ${catkin_LIBRARIES})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -g")

install(TARGETS switch_params_manager
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
