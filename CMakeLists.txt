cmake_minimum_required(VERSION 2.8.3)
project(tablebot_moveit_config)

find_package(catkin REQUIRED COMPONENTS
    actionlib
    actionlib_msgs
    cmake_modules
    control_msgs
    message_generation
    geometry_msgs
    roscpp
    moveit_ros_planning_interface
    pcl_conversions
    pcl_ros
    rail_manipulation_msgs
    sensor_msgs
    std_msgs
    std_srvs
    tf
    wpi_jaco_msgs
    )

find_package(Eigen REQUIRED)

catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(tablebot_moveit_wrapper src/tablebot_moveit.cpp include/tablebot_moveit_config/tablebot_moveit.h)
add_executable(common_actions src/common_actions.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(tablebot_moveit_wrapper ${catkin_LIBRARIES})
target_link_libraries(common_actions ${catkin_LIBRARIES})

add_dependencies(tablebot_moveit_wrapper rail_manipulation_msgs_gencpp)
add_dependencies(common_actions rail_manipulation_msgs_gencpp)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS tablebot_moveit_wrapper common_actions
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Copy header files
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
