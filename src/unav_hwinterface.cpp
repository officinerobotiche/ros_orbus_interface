/*
 * File:   unav_hwinterface.cpp
 * Author: Raffaello Bonghi
 *
 * Created on April 26, 2015, 12:00 PM
 */

#include <ros/ros.h>
#include "hardware/ORBHardware.h"
#include "controller_manager/controller_manager.h"
#include "ros/callback_queue.h"

#include <boost/chrono.hpp>

typedef boost::chrono::steady_clock time_source;

typedef struct serial_port {
    std::string name;
    int number;
} serial_port_t;

/**
 * @brief GetSerialPort Convert chars to struct serial port
 * @param c_port serial_port
 * @return serial_port_t data
 */
serial_port_t GetSerialPort(const char *c_port) {
    serial_port_t serial_port;
    std::string port(c_port);
    size_t last_index = port.find_last_not_of("0123456789");
    serial_port.name = port.substr(0, last_index + 1);
    std::istringstream(port.substr(last_index + 1)) >> serial_port.number;
    return serial_port;
}


/**
* Control loop for Husky, not realtime safe
*/
//void controlLoop(unav_hardware &orb,
//                 controller_manager::ControllerManager &cm,
//                 time_source::time_point &last_time)
//{

//  // Calculate monotonic time difference
//  time_source::time_point this_time = time_source::now();
//  boost::chrono::duration<double> elapsed_duration = this_time - last_time;
//  ros::Duration elapsed(elapsed_duration.count());
//  last_time = this_time;

//  // Process control loop
//  orb.reportLoopDuration(elapsed);
//  orb.updateJointsFromHardware();
//  cm.update(ros::Time::now(), elapsed);
//  orb.writeCommandsToHardware();
//}

/**
* Diagnostics loop for ORB boards, not realtime safe
*/
//void diagnosticLoop(orb_hardware::ORBHardware &orb)
//{
//  orb.updateDiagnostics();
//}

int main(int argc, char **argv) {

    ros::init(argc, argv, "unav_interface");
    ros::NodeHandle nh, private_nh("~");

    //Hardware information
    double control_frequency, diagnostic_frequency;
    private_nh.param<double>("control_frequency", control_frequency, 10.0);
    private_nh.param<double>("diagnostic_frequency", diagnostic_frequency, 10.0);

    //Board information
    std::string board_type_string;
    private_nh.param<std::string>("board_type", board_type_string, "");

    //Serial port configuration
    std::string serial_port_string;
    double baud_rate;
    bool arduino;
    private_nh.param<std::string>("serial_port", serial_port_string, "/dev/ttyUSB0");
    private_nh.param<double>("serial_rate", diagnostic_frequency, 115200);
    private_nh.param<bool>("serial_arduino", arduino, false);
    ParserPacket* serial;



    ROS_INFO("Open Serial %s:%d", serial_port_string.c_str(), (int)baud_rate);
    try {
        serial = new ParserPacket(serial_port_string.c_str(), baud_rate);
        //If protocol on arduino
        if(arduino) {
            int arduino = 2;
            ROS_INFO("Wait to start Arduino (%d sec) ... ", arduino);
            sleep(arduino);
            ROS_INFO("Serial Arduino started");
        }

        // Initialize robot hardware and link to controller manager
        ORBHardware *interface;

        if (board_type_string.compare("Motor Control") == 0){
            ROS_INFO("Find Controller for %s", board_type_string.c_str());
            //interface = new unav_hardware(nh, private_nh, serial, control_frequency);
        } else if (board_type_string.compare("Sensor Board") == 0) {
            ROS_INFO("Find Controller for %s", board_type_string.c_str());
            //TODO
        }else {
            ROS_INFO("Standard Controller");
            interface = new ORBHardware(nh, serial);
            ROS_INFO("Found: %s", interface->getTypeBoard().c_str());
        }

        controller_manager::ControllerManager cm(interface, nh);

        std::string name_node = ros::this_node::getName();
        ROS_INFO("Started %s", name_node.c_str());

        // Process remainder of ROS callbacks separately, mainly ControlManager related
        ros::spin();
        ROS_INFO("Find Controller for %s", board_type_string.c_str());

    } catch (std::exception &e) {
        serial->close();
        ROS_ERROR("%s", e.what());
    }
    return 0;
}
