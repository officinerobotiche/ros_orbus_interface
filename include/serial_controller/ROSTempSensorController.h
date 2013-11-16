/* 
 * File:   ROSTempSensorController.h
 * Author: raffaello
 *
 * Created on 16 November 2013, 11:49
 */

#ifndef ROSTEMPSENSORCONTROLLER_H
#define	ROSTEMPSENSORCONTROLLER_H

#include "ROSController.h"

const std::string default_laser_sharp_string = "laser_sharp";
const std::string default_base_link_string = "base_link";
const std::string laser_sharp_position_string = "sharp_pose";
const std::string default_sensor_string = "sensor";
const std::string default_parameter_string = "parameter_sensor";

const std::string enable_sensors = "enable_sensors";
const std::string enable_autosend = "enable_autosend";

class ROSTempSensorController : public ROSController {
public:
    ROSTempSensorController(std::string name_node, const ros::NodeHandle& nh, ParserPacket* serial);
    virtual ~ROSTempSensorController();
private:
    
    ros::Publisher pub_laser_sharp, pub_sensors;
    
    void sensorPacket(const unsigned char& command, const abstract_packet_t* packet);
};

#endif	/* ROSTEMPSENSORCONTROLLER_H */

