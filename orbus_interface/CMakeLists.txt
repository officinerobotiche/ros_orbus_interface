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
#                    roslint
                    serial
                    roscpp
                    sensor_msgs
                    dynamic_reconfigure
                    urdf
                    joint_limits_interface
                    orbus_msgs
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

# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/UnavPID.cfg
    cfg/UnavParameter.cfg
    cfg/UnavEncoder.cfg
    cfg/UnavBridge.cfg
    cfg/UnavEmergency.cfg
    cfg/UnavDiagnostic.cfg
)

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
        dynamic_reconfigure
        hardware_interface
        roscpp
        sensor_msgs
        joint_limits_interface
        orbus_msgs
    DEPENDS
        Boost
)

###########
## Build ##
###########
include_directories(include
                    ../or_bus_c.X/includes
                    ${Boost_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
)

add_library(or_bus
    ../or_bus_c.X/src/or_bus/or_message.c
    ../or_bus_c.X/src/or_bus/or_frame.c
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

add_definitions(-std=c++11)

#roslint_cpp(${hardware_unav_SRC})

#############
## Install ##
#############

#install(TARGETS lib_orbus_cpp hardware_unav
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

#install(DIRECTORY include/${PROJECT_NAME}/
#        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

#install(DIRECTORY launch config
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## Testing ##
#############


