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
    cfg/UnavEmergency.cfg
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
    INCLUDE_DIRS
        include
#        lib_orb_cpp/include
    LIBRARIES
#        lib_orbus_cpp
    CATKIN_DEPENDS
        diagnostic_updater
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
                    ../or_bus_c.X/includes
                    ${catkin_INCLUDE_DIRS}
)

add_library(or_bus
    ../or_bus_c.X/src/or_bus/or_message.c
    ../or_bus_c.X/src/or_bus/or_frame.c
)

#add_library(lib_orbus_cpp
#   lib_orb_cpp/src/serial_parser_packet/AsyncSerial.cpp
#   lib_orb_cpp/src/serial_parser_packet/PacketSerial.cpp
#   lib_orb_cpp/src/serial_parser_packet/ParserPacket.cpp
#)

set(hardware_unav_SRC
     src/hw_interface.cpp
     src/hardware/serial_controller.cpp
     src/hardware/GenericController.cpp
     src/hardware/uNavController.cpp
#    src/configurator/MotorPIDConfigurator.cpp
#    src/configurator/MotorParamConfigurator.cpp
#    src/configurator/MotorEmergencyConfigurator.cpp
#    src/diagnostic/MotorTask.cpp
#    src/hardware/SerialController.cpp
#    src/hardware/ORBHardware.cpp
#    src/hardware/UNAVHardware.cpp
#    src/unav_hwinterface.cpp
)

## Declare a cpp executable
add_executable(hardware_unav ${hardware_unav_SRC})
target_link_libraries(hardware_unav or_bus ${catkin_LIBRARIES})

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


