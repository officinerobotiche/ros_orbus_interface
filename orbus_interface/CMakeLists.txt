cmake_minimum_required(VERSION 2.8.3)
project(orbus_interface)

set(ROS_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Debug)

## Find catkin macros and librariess
find_package(catkin REQUIRED
                COMPONENTS
                    controller_manager
                    hardware_interface
                    diagnostic_updater
                    roslaunch
                    roslint
                    roscpp
                    sensor_msgs
                    dynamic_reconfigure
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
    INCLUDE_DIRS include lib_orb_cpp/include
    LIBRARIES lib_orbus_cpp
    CATKIN_DEPENDS
        diagnostic_updater
        dynamic_reconfigure
        hardware_interface
        roscpp
        sensor_msgs
    DEPENDS
        Boost
)

###########
## Build ##
###########
include_directories(include
                    lib_orb_cpp/include
                    ${Boost_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
)

add_library(lib_orbus_cpp
   lib_orb_cpp/src/AsyncSerial.cpp
   lib_orb_cpp/src/PacketSerial.cpp
   lib_orb_cpp/src/ParserPacket.cpp
)

set(hardware_unav_SRC
    src/configurator/MotorPIDConfigurator.cpp
    src/configurator/MotorParamConfigurator.cpp
    src/configurator/MotorEmergencyConfigurator.cpp
    src/hardware/ORBHardware.cpp
    src/hardware/UNAVHardware.cpp
    src/unav_hwinterface.cpp
)

## Declare a cpp executable
add_executable(hardware_unav ${hardware_unav_SRC})
target_link_libraries(hardware_unav lib_orbus_cpp ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(hardware_unav hardware_unav_gencpp)

roslint_cpp(${hardware_unav_SRC})

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


