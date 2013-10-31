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

    //Open Publisher
    pub_laser_sharp_ = nh_.advertise<sensor_msgs::LaserScan>("/" + name_node + "/" + default_laser_sharp_string, NUMBER_PUBLISHER,
            boost::bind(&ROSSensorController::connectCallback, this, _1));

    //Open Subscriber

    //Open Service

    //TODO
    count = 0;
    timer_ = nh_.createTimer(ros::Duration(0.1), &ROSSensorController::timerCallback, this);
//    timer_.start();
}

ROSSensorController::ROSSensorController(const ROSSensorController& orig) {
}

ROSSensorController::~ROSSensorController() {
}

void ROSSensorController::loadParameter() {
    if (nh_.hasParam(name_node_ + "/tf/" + default_base_link_string)) {
        nh_.getParam(name_node_ + "/tf/" + default_base_link_string, base_link_string_);
    } else {
        nh_.setParam(name_node_ + "/tf/" + default_base_link_string, default_base_link_string);
    }
    if (nh_.hasParam(name_node_ + "/tf/" + default_laser_sharp_string)) {
        nh_.getParam(name_node_ + "/tf/" + default_laser_sharp_string, laser_sharp_string_);
    } else {
        nh_.setParam(name_node_ + "/tf/" + default_laser_sharp_string, default_laser_sharp_string);
    }
    if (nh_.hasParam(name_node_ + "/tf/" + laser_sharp_position_string)) {
        double x, y, z, theta;
        nh_.getParam(name_node_ + "/tf/" + laser_sharp_position_string + "/x", x);
        nh_.getParam(name_node_ + "/tf/" + laser_sharp_position_string + "/y", y);
        nh_.getParam(name_node_ + "/tf/" + laser_sharp_position_string + "/z", z);
        nh_.getParam(name_node_ + "/tf/" + laser_sharp_position_string + "/theta", theta);
        pose_laser_sharp_ = tf::Vector3(x, y, z);
        angle_laser_sharp_ = tf::createQuaternionFromYaw(theta);
    } else {
        nh_.setParam(name_node_ + "/tf/" + laser_sharp_position_string + "/x", 0.0);
        nh_.setParam(name_node_ + "/tf/" + laser_sharp_position_string + "/y", 0.0);
        nh_.setParam(name_node_ + "/tf/" + laser_sharp_position_string + "/z", 0.0);
        nh_.setParam(name_node_ + "/tf/" + laser_sharp_position_string + "/theta", 0.0);
        pose_laser_sharp_ = tf::Vector3(0.0, 0.0, 0.0);
        angle_laser_sharp_ = tf::Quaternion(0, 0, 0, 1);
    }
}

void ROSSensorController::actionAsync(packet_t packet) {
    ROS_INFO("ROS Sensor Controller Async");
}

void ROSSensorController::connectCallback(const ros::SingleSubscriberPublisher& pub) {
    ROS_INFO("Connect: %s", pub.getSubscriberName().c_str());
//    timer_.start();
}

void ROSSensorController::sendLaserSharp(infrared_t infrared) {

    double laser_frequency = 40;
    
    ros::Time scan_time = ros::Time::now();
    //populate the LaserScan message
    sensor_msgs::LaserScan scan;
    scan.header.stamp = scan_time;
    scan.header.frame_id = laser_sharp_string_;
    scan.angle_min = -M_PI/2;
    scan.angle_max = M_PI/2 + (M_PI) / (NUMBER_INFRARED);
    scan.angle_increment = (M_PI) / (NUMBER_INFRARED);
    scan.time_increment = (1 / laser_frequency) / (NUMBER_INFRARED);
    scan.range_min = 0.0;
    scan.range_max = 100.0;

    scan.ranges.resize(NUMBER_INFRARED);
    scan.intensities.resize(NUMBER_INFRARED);
    for (unsigned int i = 0; i < NUMBER_INFRARED; ++i) {
        scan.ranges[i] = ((float) infrared.infrared[i]) / 10;
        //TODO verify
        scan.intensities[i] = infrared.infrared[i];
    }

    pub_laser_sharp_.publish(scan);

    broadcaster_.sendTransform(
            tf::StampedTransform(
            tf::Transform(angle_laser_sharp_, pose_laser_sharp_),
            scan_time, base_link_string_, laser_sharp_string_));
}

void ROSSensorController::timerCallback(const ros::TimerEvent& event) {
    infrared_t infrared;
    //generate some fake data for our laser scan
    for (unsigned int i = 0; i < NUMBER_INFRARED; ++i) {
        infrared.infrared[i] = count;
    }
    ++count;
    if (count == 10) count = 0;

//    ROS_INFO("Send fake sharp");
    sendLaserSharp(infrared);
}