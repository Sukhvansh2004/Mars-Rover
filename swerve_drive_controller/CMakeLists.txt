cmake_minimum_required(VERSION 3.0.2)
project(swerve_drive_controller)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  hardware_interface
  nav_msgs
  pluginlib
  roscpp
  geometry_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES swerve_drive_controller
  CATKIN_DEPENDS controller_interface hardware_interface nav_msgs pluginlib roscpp geometry_msgs tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(swerve_drive_controller
  src/swerve_drive_controller.cpp
)

target_link_libraries(swerve_drive_controller
  ${catkin_LIBRARIES}
)

install(TARGETS swerve_drive_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES plugins/swerve_drive_controller_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
