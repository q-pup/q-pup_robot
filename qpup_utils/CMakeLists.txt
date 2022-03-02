cmake_minimum_required(VERSION 3.10.2)
project(qpup_utils)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif ()

# Load catkin and package dependencies
find_package(catkin REQUIRED COMPONENTS
    roscpp
    realtime_tools
    )

# Declare a catkin package
catkin_package(
    INCLUDE_DIRS
    include
    LIBRARIES
    ${PROJECT_NAME}
    CATKIN_DEPENDS
    roscpp
)

###########
## Build ##
###########

# Specify additional locations of header files
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

# Declare and link the C++ Library
add_library(${PROJECT_NAME}
    src/qpup_params.cpp
    src/qpup_utils.cpp
    )
target_compile_features(${PROJECT_NAME} PUBLIC c_std_11 cxx_std_14)

add_library(qpup-can-gen
    generated/qpup_can_database.c
    )
target_compile_features(qpup-can-gen PUBLIC c_std_11 cxx_std_14)

add_library(qpup-can
    src/qpup_can.cpp
    )
target_compile_features(qpup-can PUBLIC c_std_11 cxx_std_17)
target_link_libraries(qpup-can qpup-can-gen)

#############
## Install ##
#############

# Install libraries
install(TARGETS ${PROJECT_NAME} qpup-can
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
    )

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
    )
