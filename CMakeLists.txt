cmake_minimum_required(VERSION 2.8.3)
project(ros_serial_bridge)

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



###################################
## catkin specific configuration ##
###################################
catkin_package(
#    INCLUDE_DIRS include
#    LIBRARIES serial_bridge
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
                    ${Boost_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
)

#### OLD ####
#set(serial_SRC
#    src/async_serial/AsyncSerial.cpp#
#    src/async_serial/PacketSerial.cpp
#    src/async_serial/ParserPacket.cpp
#    src/serial_controller/ROSController.cpp
#    src/serial_controller/ROSMotionController.cpp
#    src/serial_controller/ROSSensorController.cpp
#    src/serial_bridge_node.cpp
#)
## Declare a cpp executable
#add_executable(serial_bridge_node ${serial_SRC})
#target_link_libraries(serial_bridge_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#add_dependencies(serial_bridge_node serial_bridge_gencpp)

set(hardware_unav_SRC
    src/async_serial/AsyncSerial.cpp
    src/async_serial/PacketSerial.cpp
    src/async_serial/ParserPacket.cpp
    src/hardware/ORBHardware.cpp
    src/hardware/UNAVHardware.cpp
    src/unav_hwinterface.cpp
)

## Declare a cpp executable
add_executable(hardware_unav ${hardware_unav_SRC})
target_link_libraries(hardware_unav ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(hardware_unav hardware_unav_gencpp)

roslint_cpp(${hardware_unav_SRC})

roslaunch_add_file_check(launch)

#############
## Install ##
#############

install(TARGETS hardware_unav
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## Testing ##
#############


