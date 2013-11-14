/* 
 * File:   ROSController.h
 * Author: raffaello
 *
 * Created on 13 November 2013, 10:33
 */

#ifndef ROSCONTROLLER_H
#define	ROSCONTROLLER_H

#include <ros/ros.h>
#include <serial_bridge/Service.h>
#include "../async_serial/ParserPacket.h"

class ROSController {
public:
    ROSController(std::string name_node, const ros::NodeHandle& nh, ParserPacket* serial);

    virtual ~ROSController();
private:
    //Initialization object
    std::string name_node_; //Name for topics, params, services
    ros::NodeHandle nh_; //NameSpace for bridge controller
    ParserPacket* serial_; //Serial object to comunicate with PIC device
    ros::ServiceServer srv_board;
    
    std::string getNameBoard();
    void asyncPacket(const packet_t* packet);
    bool service_Callback(serial_bridge::Service::Request &req, serial_bridge::Service::Response &msg);
};

#endif	/* ROSCONTROLLER_H */

