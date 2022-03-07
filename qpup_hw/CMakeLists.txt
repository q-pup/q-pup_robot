cmake_minimum_required(VERSION 3.0.2)
project(qpup_hw)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif ()

find_package(catkin REQUIRED COMPONENTS
        hardware_interface
        joint_limits_interface
        qpup_utils
        roscpp
        tf2
        transmission_interface
        )
find_package(Threads REQUIRED)

catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        ${PROJECT_NAME}
        CATKIN_DEPENDS
        hardware_interface
        joint_limits_interface
        qpup_utils
        roscpp
        tf2
        transmission_interface
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    src/${PROJECT_NAME}.cpp
    src/${PROJECT_NAME}_real.cpp
    )
target_compile_features(${PROJECT_NAME} PUBLIC c_std_11 cxx_std_17)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}_navx
        src/navx/AHRS.cpp
        src/navx/ContinuousAngleTracker.cpp
        src/navx/InertialDataIntegrator.cpp
        src/navx/OffsetTracker.cpp
        src/navx/SerialIO.cpp
)
target_link_libraries(${PROJECT_NAME}_navx ${CMAKE_THREAD_LIBS_INIT})

add_executable(navx_test
        src/navx/TimeStamp.cpp
)
target_link_libraries(navx_test ${PROJECT_NAME}_navx)


install(TARGETS
    ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
    )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING
        PATTERN "*.h"
        PATTERN "*.hpp"
        PATTERN ".svn" EXCLUDE
        )

install(FILES ${PROJECT_NAME}_plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
