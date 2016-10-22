cmake_minimum_required(VERSION 2.8.3)
project(orbus_interface)

#########################################################
## Options
option( DEBUG_ACTIVE "Enable Debug build" ON )

if(DEBUG_ACTIVE)
    MESSAGE( "Debug compilation active" )
    set(ROS_BUILD_TYPE Debug)
    set(CMAKE_BUILD_TYPE Debug)
else()
    MESSAGE( "Release compilation active" )
    set(ROS_BUILD_TYPE Release)
    set(CMAKE_BUILD_TYPE Release)
endif()
#########################################################

## Find catkin macros and librariess
find_package(catkin REQUIRED
                COMPONENTS
                    controller_manager
                    hardware_interface
                    diagnostic_updater
                    roslaunch
                    serial
                    roscpp
                    sensor_msgs
                    std_srvs
                    std_msgs
                    dynamic_reconfigure
                    joint_limits_interface
                    genmsg
                    urdf
)
## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED
                COMPONENTS
                    chrono
                    system
                    thread
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
    FILES
    BoardTime.msg
    ControlStatus.msg
    MotorStatus.msg
    Peripheral.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   Service.srv
   GPIO.srv
)

# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/UnavPID.cfg
    cfg/UnavParameter.cfg
    cfg/UnavEncoder.cfg
    cfg/UnavBridge.cfg
    cfg/UnavEmergency.cfg
    cfg/UnavDiagnostic.cfg
    cfg/UnavLimits.cfg
)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

###################################
## catkin specific configuration ##
###################################
catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        or_bus
    CATKIN_DEPENDS
        diagnostic_updater
        std_msgs
        std_srvs
        dynamic_reconfigure
        hardware_interface
        roscpp
        sensor_msgs
        joint_limits_interface
    DEPENDS
        Boost
)

###########
## Build ##
###########
include_directories(include
                    or_bus_c.X/includes
                    ${Boost_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
)

add_library(or_bus
    or_bus_c.X/src/or_bus/or_message.c
    or_bus_c.X/src/or_bus/or_frame.c
)

set(hardware_unav_SRC
    src/hw_interface.cpp
    src/hardware/serial_controller.cpp
    src/hardware/GenericInterface.cpp
    src/hardware/uNavInterface.cpp
    src/hardware/Motor.cpp
    src/configurator/GenericConfigurator.cpp
    src/configurator/MotorPIDConfigurator.cpp
    src/configurator/MotorParamConfigurator.cpp
    src/configurator/MotorEmergencyConfigurator.cpp
    src/configurator/MotorDiagnosticConfigurator.cpp
)

## Declare a cpp executable
add_executable(unav_node ${hardware_unav_SRC})
target_link_libraries(unav_node or_bus ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(unav_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#############
## Install ##
#############

# Mark executables and/or libraries for installation
 install(TARGETS or_bus unav_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

# Mark cpp header files for installation
 install(DIRECTORY include/ or_bus_c.X/includes/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".git" EXCLUDE
 )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#install(DIRECTORY launch config
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-unav test/unav_node_test.cpp)
# if(TARGET ${PROJECT_NAME}-unav)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)


