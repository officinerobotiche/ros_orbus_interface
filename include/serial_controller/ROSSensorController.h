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
};

#endif	/* ROSSENSORCONTROLLER_H */

