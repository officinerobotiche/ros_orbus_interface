/* 
 * File:   ROSSensorController.h
 * Author: raffaello
 *
 * Created on 25 October 2013, 13:27
 */

#ifndef ROSSENSORCONTROLLER_H
#define	ROSSENSORCONTROLLER_H

#include "AbstractROSController.h"

#include <sensor_msgs/LaserScan.h>

#define NUMBER_PUBLISHER 10
const std::string laser_sharp_string = "laser_sharp";

class ROSSensorController : public AbstractROSController {
public:
    ROSSensorController(std::string name_node, const ros::NodeHandle& nh, Serial* serial, ServiceSerial* service_serial);
    ROSSensorController(const ROSSensorController& orig);
    virtual ~ROSSensorController();
    
    void loadParameter();
private:
    //Initialization object
    std::string name_node_; //Name for topics, params, services
    ros::NodeHandle nh_; //NameSpace for bridge controller
    Serial* serial_; //Serial object to comunicate with PIC device
    ServiceSerial* service_serial_;     //Service with board
    
    ros::Publisher pub_laser_sharp_;
    
    void actionAsync(packet_t packet);
    void connectCallback(const ros::SingleSubscriberPublisher& pub);
};

#endif	/* ROSSENSORCONTROLLER_H */

