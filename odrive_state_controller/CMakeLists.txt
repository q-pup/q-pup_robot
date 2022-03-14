cmake_minimum_required(VERSION 3.16.2)
project(odrive_state_controller)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif ()

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
        controller_interface
        hardware_interface
        odrive_state_msgs
        pluginlib
        qpup_hw
        realtime_tools
        roscpp
        )

# Declare a catkin package
catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        ${PROJECT_NAME}
        CATKIN_DEPENDS
        controller_interface
        hardware_interface
        odrive_state_msgs
        qpup_hw
        realtime_tools
        roscpp
)


###########
## Build ##
###########

# Specify header include paths
include_directories(include ${catkin_INCLUDE_DIRS})

# Declare and link the C++ Library
add_library(${PROJECT_NAME}
        src/${PROJECT_NAME}.cpp
        )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


#############
## Install ##
#############

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        )

# Install targets
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        )

# Install plugins
install(FILES ${PROJECT_NAME}_plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )