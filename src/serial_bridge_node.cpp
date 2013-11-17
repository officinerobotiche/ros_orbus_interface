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
bool arduino = false;

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
    string name_node = ros::this_node::getName();

    int baud_rate = 115200;
    if (nh.hasParam(name_node + "/info/baud_rate")) {
        nh.getParam(name_node + "/info/baud_rate", baud_rate);
    } else {
        nh.setParam(name_node + "/info/baud_rate", baud_rate);
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
            if (arduino) {
                ROS_INFO("Wait to start Arduino (2sec) ... ");
                sleep(2);
                ROS_INFO("Serial Arduino started");
            }
            if (nh.hasParam(name_node + "/info/name_board")) {
                string param_name_board;
                nh.getParam(name_node + "/info/name_board", param_name_board);
                ROS_INFO("Find Controller for %s", param_name_board.c_str());
                if (param_name_board.compare("Motion Control") == 0)
                    controller = new ROSMotionController(name_node, nh, serial);
                if (param_name_board.compare("Navigation Board") == 0)
                    controller = new ROSSensorController(name_node, nh, serial);
            } else {
                ROS_INFO("Standard Controller");
                controller = new ROSController(name_node, nh, serial);
                ROS_INFO("Founded: %s", controller->getNameBoard().c_str());
            }
            break;
        } catch (exception &e) {
            serial->close();
            ROS_ERROR("%s", e.what());
        }
    }
    // Load parameter
    controller->loadParameter();
    ROS_INFO("Started %s", name_node.c_str());

    ros::spin();
}