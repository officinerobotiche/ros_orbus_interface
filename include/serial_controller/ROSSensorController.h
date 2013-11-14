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
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

#include <serial_bridge/Enable.h>
#include <serial_bridge/Sensor.h>

#define NUMBER_PUBLISHER 10
const std::string default_laser_sharp_string = "laser_sharp";
const std::string default_base_link_string = "base_link";
const std::string laser_sharp_position_string = "sharp_pose";
const std::string default_sensor_string = "sensor";
const std::string default_parameter_string = "parameter_sensor";

const std::string enable_sensors = "enable_sensors";
const std::string enable_autosend = "enable_autosend";

class ROSSensorController : public AbstractROSController {
public:
    ROSSensorController(std::string name_node, const ros::NodeHandle& nh, ParserPacket* serial);
    ROSSensorController(const ROSSensorController& orig);
    virtual ~ROSSensorController();

    void quit(int sig);
    void loadParameter();
private:
    //Initialization object
    std::string name_node_; //Name for topics, params, services
    ros::NodeHandle nh_; //NameSpace for bridge controller
    ParserPacket* serial_; //Serial object to comunicate with PIC device

    bool dynamic_update_;
    autosend_t autosend_;
    ros::Publisher pub_laser_sharp_;
    tf::TransformBroadcaster broadcaster_;
    tf::Vector3 pose_laser_sharp_;
    tf::Quaternion angle_laser_sharp_;
    
    enable_sensor_t enable_sensor_;
    ros::Subscriber sub_enable_;
    ros::Publisher pub_sensors_;
    ros::ServiceServer srv_parameter_;
    
    std::string base_link_string_, laser_sharp_string_;
    double sharp_angle_min_, sharp_angle_max_, sharp_angle_increment_,
    sharp_time_increment_, sharp_range_min_, sharp_range_max_, sharp_distance_center_;

    void parser(ros::Duration time, std::vector<information_packet_t> serial_packet);
    
    parameter_sensor_t getParameter();
    void actionAsync(const packet_t* packet);
    void connectCallback(const ros::SingleSubscriberPublisher& pub);
    void enableCallback(const serial_bridge::Enable::ConstPtr &msg);
    void enableAutoSendCallback(const serial_bridge::Enable::ConstPtr &msg);
    bool parameterCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
    
    void sendLaserSharp(infrared_t infrared);
    //TODO to remove
    ros::Timer timer_;
    int count;
    void timerCallback(const ros::TimerEvent& event);
};

#endif	/* ROSSENSORCONTROLLER_H */

