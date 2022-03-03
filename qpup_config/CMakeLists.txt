cmake_minimum_required(VERSION 3.0.2)
project(qpup_config)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED)

# Declare a catkin package
catkin_package()


#############
## Install ##
#############

# Install config, launch, map, urdf, & world files
install(DIRECTORY config launch maps urdf worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
