cmake_minimum_required(VERSION 3.10.2)
project(qpup_utils)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# Load catkin and package dependencies
find_package(catkin REQUIRED COMPONENTS
    realtime_tools
    roscpp
    )

# Declare a catkin package
catkin_package(
    INCLUDE_DIRS
    include
    LIBRARIES
    ${PROJECT_NAME}
    qpup-can
    qpup-can-gen
    CATKIN_DEPENDS
    realtime_tools
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
    )
target_compile_features(${PROJECT_NAME} PUBLIC c_std_11 cxx_std_14)

add_library(qpup-can-gen
    src/qpup_can_generated.c
    )
target_compile_features(qpup-can-gen PUBLIC c_std_11 cxx_std_14)

add_library(qpup-can
    src/qpup_can.cpp
    )
target_compile_features(qpup-can PUBLIC c_std_11 cxx_std_17)
target_link_libraries(qpup-can ${PROJECT_NAME} qpup-can-gen)

# TODO REMOVE:
add_executable(
    test-node
    src/test_node.cpp
)
target_link_libraries(test-node qpup-can ${PROJECT_NAME} ${catkin_LIBRARIES})

#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(odrive-can-test
      test/odrive_can.test
      test/odrive_can.cpp)
  target_compile_features(odrive-can-test PUBLIC c_std_11 cxx_std_17)
  target_link_libraries(odrive-can-test qpup-can ${catkin_LIBRARIES})
endif ()

#############
## Install ##
#############

# Install libraries
install(
    TARGETS
    qpup-can
    qpup-can-gen
    ${PROJECT_NAME}

    ARCHIVE DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION
    ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
    )

# Install Scripts
catkin_install_python(PROGRAMS
    scripts/generate_dbc.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
