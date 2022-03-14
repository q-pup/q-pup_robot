cmake_minimum_required(VERSION 3.16.3)
project(odrive_state_msgs)

find_package(catkin REQUIRED COMPONENTS
        message_generation
        std_msgs
        )

add_message_files(
        FILES
        OdriveState.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
)

catkin_package(
        CATKIN_DEPENDS
        message_runtime
        std_msgs
)