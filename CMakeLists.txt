cmake_minimum_required(VERSION 2.8.3)
project(serial_bridge)

set(ROS_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Debug)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  tf
  message_generation
  geometry_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system thread)

find_package( PkgConfig REQUIRED )
pkg_check_modules ( ncurses++ REQUIRED ncurses++ )

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
    FILES
    Velocity.msg
    Pose.msg
    Enable.msg
    Motor.msg
    Process.msg
    Sensor.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   Update.srv
   Convert.srv
   Service.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES serial_bridge
   CATKIN_DEPENDS geometry_msgs message_runtime std_msgs std_srvs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
set(teleop_INC_DIRS
    src/keybrd_teleop/include/
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${teleop_INC_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

## Declare a cpp library
# add_library(serial_bridge
#   src/${PROJECT_NAME}/serial_bridge.cpp
# )

set(serial_SRC
    src/async_serial/AsyncSerial.cpp
    src/async_serial/PacketSerial.cpp
    src/async_serial/ParserPacket.cpp
    src/serial_controller/ROSController.cpp
    src/serial_controller/ROSMotionController.cpp
    src/serial_controller/ROSSensorController.cpp
    src/serial_bridge_node.cpp
)

set(drive_teleop_SRC
    src/drive_bridge_node.cpp
    src/drive_bridge/Keyboard.cpp
)

set(keybrd_teleop_SRC
    src/keybrd_teleop_node.cpp
    src/keybrd_teleop/src/teleop_key.cpp
)

## Declare a cpp executable
add_executable(serial_bridge_node ${serial_SRC})
target_link_libraries(serial_bridge_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(serial_bridge_node serial_bridge_gencpp)

add_executable(drive_bridge_node ${drive_teleop_SRC})
target_link_libraries(drive_bridge_node ${catkin_LIBRARIES})
add_dependencies(drive_bridge_node serial_bridge_gencpp)

add_executable(keybrd_teleop_key_node ${keybrd_teleop_SRC})
target_link_libraries(keybrd_teleop_key_node ${catkin_LIBRARIES} ncurses)
add_dependencies(keybrd_teleop_key_node	${PROJECT_NAME}_generate_messages_cpp)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS serial_bridge serial_bridge_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_serial_bridge.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
