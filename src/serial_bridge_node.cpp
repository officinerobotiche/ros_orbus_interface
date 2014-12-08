/* 
 * File:   serial_bridge_node.cpp
 * Author: Raffaello Bonghi
 *
 * Created on June 7, 2013, 4:16 PM
 */

#include <ros/ros.h>
#include "async_serial/ParserPacket.h"
#include "serial_controller/ROSController.h"
#include "serial_controller/ROSMotionController.h"
#include "serial_controller/ROSSensorController.h"

using namespace std;

typedef struct serial_port {
    string name;
    int number;
} serial_port_t;

serial_port_t GetSerialPort(char *c_port) {
    serial_port_t serial_port;
    string port(c_port);
    size_t last_index = port.find_last_not_of("0123456789");
    serial_port.name = port.substr(0, last_index + 1);
    istringstream(port.substr(last_index + 1)) >> serial_port.number;
    return serial_port;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "serial_bridge");
    serial_port_t serial_port1, serial_port2;
    switch (argc) {
        case 1:
            ROS_ERROR("need serial port name as argument");
            return -1;
            break;
        case 2:
            serial_port1 = GetSerialPort(argv[1]);
            serial_port2 = serial_port1;
            break;
        default:
            serial_port1 = GetSerialPort(argv[1]);
            serial_port2 = GetSerialPort(argv[2]);
            if (serial_port1.name.compare(serial_port2.name) != 0) {
                ROS_ERROR("Need same serial port type");
                return -1;
            }
            break;
    }

    ros::NodeHandle nh;

    int baud_rate = 115200;
    if (nh.hasParam("info/baud_rate")) {
        nh.getParam("info/baud_rate", baud_rate);
    } else {
        nh.setParam("info/baud_rate", baud_rate);
    }

    ParserPacket* serial;
    ROSController* controller;

    for (int i = serial_port1.number; i <= serial_port2.number; ++i) {
        stringstream number; //create a stringstream
        number << i; //add number to the stream
        ROS_INFO("Open Serial %s%d:%d", serial_port1.name.c_str(), i, baud_rate);
        try {
            serial = new ParserPacket(serial_port1.name + number.str(), baud_rate);
            //If protocoll on arduino
            if (nh.hasParam("info/arduino")) {
                int arduino = 2;
                nh.getParam("info/arduino", arduino);
                ROS_INFO("Wait to start Arduino (%d sec) ... ",arduino);
                sleep(arduino);
                ROS_INFO("Serial Arduino started");
            }
            if (nh.hasParam("info/type_board")) {
                string param_type_board;
                nh.getParam("info/type_board", param_type_board);
                ROS_INFO("Find Controller for %s", param_type_board.c_str());
                if (param_type_board.compare("Motor Control") == 0)
                    controller = new ROSMotionController(nh, serial);
                else if (param_type_board.compare("Sensor Board") == 0)
                    controller = new ROSSensorController(nh, serial);
                else 
                    controller = new ROSController(nh, serial);
            } else {
                ROS_INFO("Standard Controller");
                controller = new ROSController(nh, serial);
                ROS_INFO("Founded: %s", controller->getTypeBoard().c_str());
            }
            break;
        } catch (exception &e) {
            serial->close();
            ROS_ERROR("%s", e.what());
        }
    }
    // Load parameter
    controller->loadParameter();
    string name_node = ros::this_node::getName();
    ROS_INFO("Started %s", name_node.c_str());

    ros::spin();
}