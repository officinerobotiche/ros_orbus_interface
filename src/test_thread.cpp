
#include <ros/ros.h>
#include "async_serial/ParserPacket.h"
#include "serial_controller/ROSController.h"
#include "serial_controller/ROSMotionController.h"
#include "serial_controller/ROSTempSensorController.h"

using namespace std;

int main(int argc, char **argv) {

    ros::init(argc, argv, "serial_bridge");

    ros::NodeHandle nh;

    int baud_rate = 115200;
    string serial_port = "/dev/ttyUSB0";
    string name_node = ros::this_node::getName();
    ParserPacket* serial;
    ROSController* controller;
    
    ROS_INFO("Open Serial %s:%d", serial_port.c_str(), baud_rate);
    try {
        serial = new ParserPacket(serial_port, baud_rate);
        if (nh.hasParam(name_node + "/name_board")) {
            string param_name_board;
            nh.getParam(name_node + "/name_board", param_name_board);
            ROS_INFO("Find Controller for %s", param_name_board.c_str());
            if (param_name_board.compare("Motion Control") == 0)
                controller = new ROSMotionController(name_node, nh, serial);
            if (param_name_board.compare("Navigation Board") == 0)
                controller = new ROSTempSensorController(name_node, nh, serial);
        } else {
            ROS_INFO("Standard Controller");
            controller = new ROSController(name_node, nh, serial);
            ROS_INFO("Founded: %s", controller->getNameBoard().c_str());
        }
    } catch (exception &e) {
        serial->close();
        ROS_ERROR("%s", e.what());
        return -1;
    }
    controller->loadParameter();

    ros::spin();
}