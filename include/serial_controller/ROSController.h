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
    
    void loadParameter();
    void updatePacket(std::vector<information_packet_t>* list);
private:
    //Initialization object
    std::string name_node_; //Name for topics, params, services
    ros::NodeHandle nh_; //NameSpace for bridge controller
    ParserPacket* serial_; //Serial object to comunicate with PIC device
    ros::Timer timer_;
    ros::ServiceServer srv_board;
    ros::Publisher pub_time_process;
    
    std::string name_board, version, name_author, compiled;
    double step_timer, tm_mill, k_time;
    error_pkg_t error_serial;
    
    void timerCallback(const ros::TimerEvent& event);
    void connectCallback(const ros::SingleSubscriberPublisher& pub);
    
    float getTimeProcess(float process_time);
    void defaultPacket(const unsigned char& command, const abstract_packet_t* packet);
    
    information_packet_t encodeServices(char command, unsigned char* buffer=NULL, size_t len=0);
    void resetBoard(unsigned int repeat=3);
    void decodeServices(const char command, const unsigned char* buffer);
    
    bool service_Callback(serial_bridge::Service::Request &req, serial_bridge::Service::Response &msg);
};

#endif	/* ROSCONTROLLER_H */

