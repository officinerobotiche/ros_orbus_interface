/* 
 * File:   ROSSensorController.cpp
 * Author: raffaello
 * 
 * Created on 25 October 2013, 13:27
 */

#include "serial_controller/ROSSensorController.h"

ROSSensorController::ROSSensorController(std::string name_node, const ros::NodeHandle& nh, Serial* serial, ServiceSerial* service_serial)
: nh_(nh) {
    name_node_ = name_node; // Initialize node name
    this->serial_ = serial; // Initialize serial port
    this->service_serial_ = service_serial; //Initialize service with serial
    serial_->asyncPacket(&ROSSensorController::actionAsync, this);

    //Create publishers
    pub_laser_sharp_ = nh_.advertise<sensor_msgs::LaserScan>("/" + name_node + "/" + laser_sharp_string, NUMBER_PUBLISHER,
            boost::bind(&ROSSensorController::connectCallback, this, _1));
}

ROSSensorController::ROSSensorController(const ROSSensorController& orig) {
}

ROSSensorController::~ROSSensorController() {
}

void ROSSensorController::loadParameter() {

}

void ROSSensorController::actionAsync(packet_t packet) {
    ROS_INFO("ROS Sensor Controller Async");
}

void ROSSensorController::connectCallback(const ros::SingleSubscriberPublisher& pub) {
    ROS_INFO("Connect: %s", pub.getSubscriberName().c_str());
    
}