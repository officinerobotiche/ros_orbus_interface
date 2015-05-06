/* 
 * File:   ROSSensorController.h
 * Author: raffaello
 *
 * Created on 16 November 2013, 11:49
 */

#ifndef ROSTEMPSENSORCONTROLLER_H
#define	ROSTEMPSENSORCONTROLLER_H

#include "ROSController.h"
#include <ros_serial_bridge/Enable.h>
#include <tf/transform_broadcaster.h>

const std::string default_laser_sharp_string = "laser_sharp";
const std::string default_base_link_string = "base_link";
const std::string laser_sharp_position_string = "sharp_pose";
const std::string default_temperature_string = "temperature";
const std::string default_sensor_string = "other_sensors";
const std::string default_parameter_string = "parameter_sensor";

const std::string enable_sensors = "enable_sensors";
const std::string enable_autosend = "enable_autosend";

class ROSSensorController : public ROSController {
public:
    ROSSensorController(const ros::NodeHandle& nh, ParserPacket* serial);
    virtual ~ROSSensorController();
private:

    ros::Publisher pub_laser_sharp, pub_temperature, pub_sensors;
    ros::Subscriber sub_enable;
    ros::ServiceServer srv_parameter;
    tf::TransformBroadcaster broadcaster;
    tf::Vector3 pose_laser_sharp;
    tf::Quaternion angle_laser_sharp;

    autosend_t autosend;
    enable_sensor_t enable;
    bool dynamic_update;
    std::string base_link_string_, laser_sharp_string_;
    double sharp_angle_min_, sharp_angle_max_, sharp_angle_increment_,
    sharp_time_increment_, sharp_range_min_, sharp_range_max_, sharp_distance_center_;
    
    bool compareAutosend(autosend_t autosend1, autosend_t autosend2);
    void updatePacket(std::vector<information_packet_t>* list_send);
    void addParameter(std::vector<information_packet_t>* list_send);
    bool aliveOperation(const ros::TimerEvent& event, std::vector<information_packet_t>* list_send);
    void sensorPacket(const unsigned char& command, const abstract_message_u* packet);
    void sendLaserSharp(infrared_t infrared);

    void enableCallback(const ros_serial_bridge::Enable::ConstPtr &msg);
    bool parameterCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);

    parameter_sensor_t getParameter();
};

#endif	/* ROSTEMPSENSORCONTROLLER_H */

