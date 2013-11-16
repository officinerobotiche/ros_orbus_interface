/* 
 * File:   ROSTempSensorController.h
 * Author: raffaello
 *
 * Created on 16 November 2013, 11:49
 */

#ifndef ROSTEMPSENSORCONTROLLER_H
#define	ROSTEMPSENSORCONTROLLER_H

#include "ROSController.h"
#include <serial_bridge/Enable.h>
#include <tf/transform_broadcaster.h>

const std::string default_laser_sharp_string = "laser_sharp";
const std::string default_base_link_string = "base_link";
const std::string laser_sharp_position_string = "sharp_pose";
const std::string default_sensor_string = "other_sensors";
const std::string default_parameter_string = "parameter_sensor";

const std::string enable_sensors = "enable_sensors";
const std::string enable_autosend = "enable_autosend";

class ROSTempSensorController : public ROSController {
public:
    ROSTempSensorController(std::string name_node, const ros::NodeHandle& nh, ParserPacket* serial);
    virtual ~ROSTempSensorController();
private:

    ros::Publisher pub_laser_sharp, pub_sensors;
    ros::Subscriber sub_enable;
    ros::ServiceServer srv_parameter;
    tf::TransformBroadcaster broadcaster;
    tf::Vector3 pose_laser_sharp;
    tf::Quaternion angle_laser_sharp;

    bool dynamic_update;
    std::string base_link_string_, laser_sharp_string_;
    double sharp_angle_min_, sharp_angle_max_, sharp_angle_increment_,
    sharp_time_increment_, sharp_range_min_, sharp_range_max_, sharp_distance_center_;

    void addParameter(std::vector<information_packet_t>* list_send);
    void sensorPacket(const unsigned char& command, const abstract_packet_t* packet);
    void sendLaserSharp(infrared_t infrared);

    void enableCallback(const serial_bridge::Enable::ConstPtr &msg);
    bool parameterCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);

    parameter_sensor_t getParameter();
};

#endif	/* ROSTEMPSENSORCONTROLLER_H */

