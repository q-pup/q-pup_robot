cmake_minimum_required(VERSION 3.0.2)
project(qpup_vision)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  actionlib
  ar_track_alvar
  geometry_msgs
  move_base_msgs
  roscpp
  std_msgs
  tf2_geometry_msgs
  tf2_ros
)

# Declare a catkin package
catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    actionlib
    ar_track_alvar
    geometry_msgs
    move_base_msgs
    roscpp
    std_msgs
    tf2_geometry_msgs
    tf2_ros
)


###########
## Build ##
###########

# Specify header include paths
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(
  person_tracker
  src/person_tracker.cpp
)
target_link_libraries(person_tracker ${catkin_LIBRARIES})


#############
## Install ##
#############

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


# Install targets
install(TARGETS person_tracker
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Install launch & rviz files
install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
