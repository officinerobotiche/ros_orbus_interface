/* 
 * File:   ROSSensorController.cpp
 * Author: raffaello
 * 
 * Created on 25 October 2013, 13:27
 */

#include "serial_controller/ROSSensorController.h"

ROSSensorController::ROSSensorController(std::string name_node, const ros::NodeHandle& nh, Serial* serial)
: nh_(nh) {
    name_node_ = name_node; // Initialize node name
    this->serial_ = serial; // Initialize serial port
    serial_->asyncPacket(&ROSSensorController::actionAsync, this);

    //Open Publisher
    pub_laser_sharp_ = nh_.advertise<sensor_msgs::LaserScan>("/" + name_node + "/" + default_laser_sharp_string, NUMBER_PUBLISHER,
            boost::bind(&ROSSensorController::connectCallback, this, _1));

    //Open Subscriber
    //-Command
    sub_enable_ = nh_.subscribe("/" + name_node + "/" + command_string + "/" + enable_sensors, 1, &ROSSensorController::enableCallback, this);

    //Open Service
    
    //Init
    enable_sensor_ = false;

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
    //Set laser scan
    if (nh_.hasParam(name_node_ + "/" + default_laser_sharp_string)) {
        nh_.getParam(name_node_ + "/" + default_laser_sharp_string + "/angle/min", sharp_angle_min_);
        nh_.getParam(name_node_ + "/" + default_laser_sharp_string + "/angle/max", sharp_angle_max_);
        nh_.getParam(name_node_ + "/" + default_laser_sharp_string + "/angle/increment", sharp_angle_increment_);
        nh_.getParam(name_node_ + "/" + default_laser_sharp_string + "/time/increment", sharp_time_increment_);
        nh_.getParam(name_node_ + "/" + default_laser_sharp_string + "/range/min", sharp_range_min_);
        nh_.getParam(name_node_ + "/" + default_laser_sharp_string + "/range/max", sharp_range_max_);
    } else {
        double laser_frequency = 40;
        nh_.setParam(name_node_ + "/" + default_laser_sharp_string + "/angle/min", -M_PI / 2);
        nh_.setParam(name_node_ + "/" + default_laser_sharp_string + "/angle/max", M_PI / 2);
        nh_.setParam(name_node_ + "/" + default_laser_sharp_string + "/angle/increment", (M_PI) / (NUMBER_INFRARED));
        nh_.setParam(name_node_ + "/" + default_laser_sharp_string + "/time/increment", (1 / laser_frequency) / (NUMBER_INFRARED));
        nh_.setParam(name_node_ + "/" + default_laser_sharp_string + "/range/min", 0.04);
        nh_.setParam(name_node_ + "/" + default_laser_sharp_string + "/range/max", 0.40);
    }
    //Read enable sensor
    packet_t send_pkg;
    send_pkg.length = 0;
    Serial::addPacket(&send_pkg, ENABLE_SENSOR, REQUEST, NULL);
    std::list<information_packet_t> serial = Serial::parsing(NULL, serial_->sendPacket(send_pkg));
    for (std::list<information_packet_t>::iterator list_iter = serial.begin(); list_iter != serial.end(); list_iter++) {
        information_packet_t packet = (*list_iter);
        if (packet.option == CHANGE) {
            switch (packet.command) {
                case ENABLE_SENSOR:
                    enable_sensor_ = packet.packet.enable_sensor;
                    ROS_INFO("Enable state: %d", packet.packet.enable_sensor);
                    break;
            }
        }
    }
}

void ROSSensorController::actionAsync(packet_t packet) {
    ROS_INFO("ROS Sensor Controller Async");
}

void ROSSensorController::connectCallback(const ros::SingleSubscriberPublisher& pub) {
    ROS_INFO("Connect: %s", pub.getSubscriberName().c_str());
    //Start regulator
    packet_t send_pkg;
    send_pkg.length = 0;
    enable_sensor_ = true;
    Serial::addPacket(&send_pkg, ENABLE_SENSOR, CHANGE, (abstract_packet_t*) & enable_sensor_);
    // TODO VERIFY THE PACKET
    Serial::parsing(NULL, serial_->sendPacket(send_pkg));
}

void ROSSensorController::sendLaserSharp(infrared_t infrared) {

    ros::Time scan_time = ros::Time::now();

    //Update information position laser sharp
    double x, y, z, theta;
    nh_.getParam(name_node_ + "/tf/" + laser_sharp_position_string + "/x", x);
    nh_.getParam(name_node_ + "/tf/" + laser_sharp_position_string + "/y", y);
    nh_.getParam(name_node_ + "/tf/" + laser_sharp_position_string + "/z", z);
    nh_.getParam(name_node_ + "/tf/" + laser_sharp_position_string + "/theta", theta);
    pose_laser_sharp_ = tf::Vector3(x, y, z);
    angle_laser_sharp_ = tf::createQuaternionFromYaw(theta);

    broadcaster_.sendTransform(
            tf::StampedTransform(
            tf::Transform(angle_laser_sharp_, pose_laser_sharp_),
            scan_time, base_link_string_, laser_sharp_string_));

    //Update information laser sharp
    nh_.getParam(name_node_ + "/" + default_laser_sharp_string + "/angle/min", sharp_angle_min_);
    nh_.getParam(name_node_ + "/" + default_laser_sharp_string + "/angle/max", sharp_angle_max_);
    nh_.getParam(name_node_ + "/" + default_laser_sharp_string + "/angle/increment", sharp_angle_increment_);
    nh_.getParam(name_node_ + "/" + default_laser_sharp_string + "/time/increment", sharp_time_increment_);
    nh_.getParam(name_node_ + "/" + default_laser_sharp_string + "/range/min", sharp_range_min_);
    nh_.getParam(name_node_ + "/" + default_laser_sharp_string + "/range/max", sharp_range_max_);

    //populate the LaserScan message
    sensor_msgs::LaserScan scan;
    scan.header.stamp = scan_time;
    scan.header.frame_id = laser_sharp_string_;
    scan.angle_min = sharp_angle_min_;
    scan.angle_max = sharp_angle_max_;
    scan.angle_increment = sharp_angle_increment_;
    scan.time_increment = sharp_time_increment_;
    scan.range_min = sharp_range_min_;
    scan.range_max = sharp_range_max_;

    scan.ranges.resize(NUMBER_INFRARED);
    scan.intensities.resize(NUMBER_INFRARED);
    for (unsigned int i = 0; i < NUMBER_INFRARED; ++i) {
        scan.ranges[i] = ((float) infrared.infrared[i]) / 1000;
        //TODO verify
        scan.intensities[i] = ((float) infrared.infrared[i]) / 1000;
    }

    pub_laser_sharp_.publish(scan);
}

void ROSSensorController::enableCallback(const serial_bridge::Enable::ConstPtr &msg) {
    packet_t send_pkg;
    send_pkg.length = 0;
    enable_sensor_t enable = msg->enable;
    Serial::addPacket(&send_pkg, ENABLE_SENSOR, CHANGE, (abstract_packet_t*) & enable);
    // TODO VERIFY THE PACKET
    Serial::parsing(NULL, serial_->sendPacket(send_pkg));
}

void ROSSensorController::timerCallback(const ros::TimerEvent& event) {
    infrared_t infrared;
    //generate some fake data for our laser scan
    for (unsigned int i = 0; i < NUMBER_INFRARED; ++i) {
        infrared.infrared[i] = count * 100 + i * 100;
    }
    ++count;
    if (count == 10) count = 0;

    //    ROS_INFO("Send fake sharp");
    sendLaserSharp(infrared);

    //Start and stop regulator
    if ((pub_laser_sharp_.getNumSubscribers() == 0) && (enable_sensor_ == true)) {
        ROS_INFO("Stop regulator");
        packet_t send_pkg;
        send_pkg.length = 0;
        enable_sensor_ = false;
        Serial::addPacket(&send_pkg, ENABLE_SENSOR, CHANGE, (abstract_packet_t*) & enable_sensor_);
        // TODO VERIFY THE PACKET
        Serial::parsing(NULL, serial_->sendPacket(send_pkg));
    }
}