cmake_minimum_required(VERSION 2.8.3)
project(simulacion_reparacion)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roslib
  descartes_core
  descartes_moveit
  descartes_trajectory
  descartes_planner
  descartes_utilities
  trajectory_msgs
  tf
  tf_conversions
  eigen_conversions
  interactive_markers
  moveit_core
  moveit_visual_tools
  moveit_ros_planning
  moveit_ros_planning_interface
  rviz
  visualization_msgs
  tutorial_utilities REQUIRED
)

catkin_package()

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Tutorial 1
add_executable(${PROJECT_NAME}generacion_trayectorias src/generacion_trayectorias.cpp)

target_link_libraries(${PROJECT_NAME}generacion_trayectorias
 ${catkin_LIBRARIES}
)

set_target_properties(${PROJECT_NAME}generacion_trayectorias
  PROPERTIES OUTPUT_NAME generacion_trayectorias
  PREFIX "")




#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}generacion_trayectorias 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
