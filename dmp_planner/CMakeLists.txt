cmake_minimum_required(VERSION 3.0.2)
project(dmp_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pluginlib
  costmap_2d
  nav_core
  nav_msgs
  geometry_msgs
  common_msgs
  dmp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dmp_planner
  CATKIN_DEPENDS roscpp pluginlib costmap_2d nav_core nav_msgs geometry_msgs common_msgs dmp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/dmp_planner.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# Install library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(FILES dmp_planner_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
