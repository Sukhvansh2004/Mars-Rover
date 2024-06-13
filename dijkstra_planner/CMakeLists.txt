cmake_minimum_required(VERSION 3.0.2)
project(dijkstra_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pluginlib
  costmap_2d
  nav_core
  nav_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dijkstra_planner
  CATKIN_DEPENDS roscpp pluginlib costmap_2d nav_core nav_msgs geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(dijkstra_planner src/dijkstra_planner.cpp)

target_link_libraries(dijkstra_planner
  ${catkin_LIBRARIES}
)

install(TARGETS dijkstra_planner
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
