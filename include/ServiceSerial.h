/* 
 * File:   ServiceSerial.h
 * Author: raffaello
 *
 * Created on 26 October 2013, 14:43
 */

#ifndef SERVICESERIAL_H
#define	SERVICESERIAL_H

#include "Serial.h"
#include "ros/ros.h"

#include <serial_bridge/Service.h>

const std::string service_string = "service";
const std::string reset_string = "reset";
const std::string version_string = "version";
const std::string error_serial_string = "serial";

const std::string step_timer_string = "step_timer";
const std::string int_tm_mill_string = "tm_mill";
const std::string k_time_string = "k_time";
const std::string time_string = "time";

class ServiceSerial {
public:
    ServiceSerial(std::string name_node, const ros::NodeHandle& nh, Serial* serial);
    ServiceSerial(const ServiceSerial& orig);
    virtual ~ServiceSerial();
    void resetBoard(unsigned int repeat);
    std::string getNameBoard();
    std::string getAuthor();
    std::string getCompiled();
    std::string getVersion();
    std::string getErrorSerial();
    float getTimeProcess(float process_time);
private:
    std::string name_node_;
    ros::NodeHandle nh_; //NameSpace for bridge controller
    Serial* serial_; //Serial object to communicate with PIC device
    std::string name_board, version, name_author, compiled;
    ros::ServiceServer srv_board_;
    double step_timer_, tm_mill_, k_time_;
    
    abstract_packet_t getServiceSerial(std::list<information_packet_t> configuration, unsigned char command, unsigned char service_command);
    void actionAsync(packet_t packet);
    bool service_Callback(serial_bridge::Service::Request &req, serial_bridge::Service::Response &msg);
};

#endif	/* SERVICESERIAL_H */

