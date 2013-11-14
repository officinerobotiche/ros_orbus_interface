/* 
 * File:   ROSController.cpp
 * Author: raffaello
 * 
 * Created on 13 November 2013, 10:33
 */

#include "serial_controller/ROSController.h"

ROSController::ROSController(std::string name_node, const ros::NodeHandle& nh, ParserPacket* serial)
: nh_(nh), name_node_(name_node), serial_(serial) {
    serial_->setAsyncPacketCallback(&ROSController::asyncPacket, this);

    //Services
    srv_board = nh_.advertiseService(name_node + "/service_serial", &ROSController::service_Callback, this);
}

ROSController::~ROSController() {
    serial_->clearAsyncPacketCallback();
}

void ROSController::asyncPacket(const packet_t* packet) {
    ROS_INFO("I'M HERE");
}

std::string ROSController::getNameBoard() {
    return "NULL";
}

bool ROSController::service_Callback(serial_bridge::Service::Request &req, serial_bridge::Service::Response & msg) {
    if (req.name.compare("reset") == 0) {
        msg.name = "reset";
        return true;
    } else if (req.name.compare("version") == 0) {
        msg.name = "version";
        return true;
    } else if (req.name.compare("serial_info") == 0) {
        msg.name = "serial_info";
        return true;
    } else {
        msg.name = "help";
        return true;
    }
}