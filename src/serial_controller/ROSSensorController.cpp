/* 
 * File:   ROSSensorController.cpp
 * Author: raffaello
 * 
 * Created on 16 November 2013, 11:49
 */

#include "serial_controller/ROSSensorController.h"
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Temperature.h>
#include <ros_serial_bridge/Sensor.h>

#define NUMBER_PUB 10

using namespace std;

ROSSensorController::ROSSensorController(const ros::NodeHandle& nh, ParserPacket* serial)
: ROSController(nh, serial), dynamic_update(false) {

    string param_type_board = "Sensor Board";
    if (type_board.compare(param_type_board) == 0) {
        nh_.setParam("info/type_board", type_board);
    } else {
        throw (controller_exception("Other board: " + type_board));
    }

    serial->addCallback(&ROSSensorController::sensorPacket, this, HASHMAP_NAVIGATION);
    addVectorPacketRequest(&ROSSensorController::updatePacket, this);
    addParameterPacketRequest(&ROSSensorController::addParameter, this);
    addAliveOperation(&ROSSensorController::aliveOperation, this, true);

    //Open Publisher
    pub_laser_sharp = nh_.advertise<sensor_msgs::LaserScan>(default_laser_sharp_string, NUMBER_PUB,
            boost::bind(&ROSController::connectCallback, this, _1));
    pub_temperature = nh_.advertise<sensor_msgs::Temperature>(default_temperature_string, NUMBER_PUB,
            boost::bind(&ROSController::connectCallback, this, _1));
    pub_sensors = nh_.advertise<ros_serial_bridge::Sensor>(default_sensor_string, NUMBER_PUB,
            boost::bind(&ROSController::connectCallback, this, _1));
    //Open Subscriber
    //-Command
    sub_enable = nh_.subscribe("/" + enable_sensors, 1, &ROSSensorController::enableCallback, this);

    //Open Service
    srv_parameter = nh_.advertiseService(default_parameter_string, &ROSSensorController::parameterCallback, this);

    autosend.pkgs[0] = -1;
}

ROSSensorController::~ROSSensorController() {
    serial_->clearCallback(HASHMAP_NAVIGATION);
    clearVectorPacketRequest();
    clearParameterPacketRequest();
    clearAliveOperation();
}

bool ROSSensorController::compareAutosend(autosend_t autosend1, autosend_t autosend2) {
    bool compare = false;
    bool next = false;
    int i = 0;
    do {
        if (autosend1.pkgs[i] == autosend2.pkgs[i]) {
            compare = true;
        } else {
            compare = false;
        }
        if (autosend1.pkgs[i] == -1 || autosend2.pkgs[i] == -1) {
            next = false;
        }
    } while (next);
    return compare;
}

bool ROSSensorController::aliveOperation(const ros::TimerEvent& event, std::vector<information_packet_t>* list_send) {
    vector<information_packet_t> list;
    if (enable) {
        //        ROS_INFO("Enable");
        list.push_back(serial_->createDataPacket(ENABLE_SENSOR, HASHMAP_NAVIGATION, (abstract_message_u*) & enable));
    }
    if (autosend.pkgs[0] != -1) {
        list.push_back(serial_->createDataPacket(ENABLE_AUTOSEND, HASHMAP_NAVIGATION, (abstract_message_u*) & autosend));
        //        ROS_INFO("Auto send %d", ((unsigned int)autosend.pkgs[0]));
    }
    serial_->sendAsyncPacket(serial_->encoder(list));
    return true;
}

void ROSSensorController::updatePacket(std::vector<information_packet_t>* list_send) {
    autosend_t new_autosend;
    enable_sensor_t new_enable;
    unsigned int counter = 0;
    if (pub_laser_sharp.getNumSubscribers() >= 1) {
        new_enable = true;
        new_autosend.pkgs[counter++] = INFRARED;
    } else {
        new_enable = false;
    }
    if (pub_sensors.getNumSubscribers() >= 1 || pub_temperature.getNumSubscribers() >= 1) {
        //        ROS_INFO("Sensor start");
        new_autosend.pkgs[counter++] = SENSOR;
    }
    new_autosend.pkgs[counter++] = -1;
    if (new_enable != enable) {
        enable = new_enable;
        //        ROS_INFO("Update packet enable");
        list_send->push_back(serial_->createDataPacket(ENABLE_SENSOR, HASHMAP_NAVIGATION, (abstract_message_u*) & enable));
    }
    if (!compareAutosend(new_autosend, autosend)) {
        autosend = new_autosend;
        //        ROS_INFO("Update packet autosend");
        list_send->push_back(serial_->createDataPacket(ENABLE_AUTOSEND, HASHMAP_NAVIGATION, (abstract_message_u*) & autosend));
    }
}

void ROSSensorController::addParameter(std::vector<information_packet_t>* list_send) {
    if (nh_.hasParam("/tf/" + default_base_link_string)) {
        nh_.getParam("/tf/" + default_base_link_string, base_link_string_);
    } else {
        nh_.setParam("/tf/" + default_base_link_string, default_base_link_string);
    }
    if (nh_.hasParam("/tf/" + default_laser_sharp_string)) {
        nh_.getParam("/tf/" + default_laser_sharp_string, laser_sharp_string_);
    } else {
        nh_.setParam("/tf/" + default_laser_sharp_string, default_laser_sharp_string);
    }
    if (nh_.hasParam("/tf/" + laser_sharp_position_string + "/dynamic_update")) {
        nh_.getParam("/tf/" + laser_sharp_position_string + "/dynamic_update", dynamic_update);
    } else {
        nh_.setParam("/tf/" + laser_sharp_position_string + "/dynamic_update", false);
    }
    if (nh_.hasParam("/tf/" + laser_sharp_position_string)) {
        double x, y, z, theta;
        nh_.getParam("/tf/" + laser_sharp_position_string + "/x", x);
        nh_.getParam("/tf/" + laser_sharp_position_string + "/y", y);
        nh_.getParam("/tf/" + laser_sharp_position_string + "/z", z);
        nh_.getParam("/tf/" + laser_sharp_position_string + "/theta", theta);
        pose_laser_sharp = tf::Vector3(x, y, z);
        angle_laser_sharp = tf::createQuaternionFromYaw(theta);
    } else {
        nh_.setParam("/tf/" + laser_sharp_position_string + "/x", 0.0);
        nh_.setParam("/tf/" + laser_sharp_position_string + "/y", 0.0);
        nh_.setParam("/tf/" + laser_sharp_position_string + "/z", 0.0);
        nh_.setParam("/tf/" + laser_sharp_position_string + "/theta", 0.0);
        pose_laser_sharp = tf::Vector3(0.0, 0.0, 0.0);
        angle_laser_sharp = tf::Quaternion(0, 0, 0, 1);
    }
    //Set laser scan
    if (nh_.hasParam(default_laser_sharp_string)) {
        nh_.getParam(default_laser_sharp_string + "/angle/min", sharp_angle_min_);
        nh_.getParam(default_laser_sharp_string + "/angle/max", sharp_angle_max_);
        nh_.getParam(default_laser_sharp_string + "/angle/increment", sharp_angle_increment_);
        nh_.getParam(default_laser_sharp_string + "/time/increment", sharp_time_increment_);
        nh_.getParam(default_laser_sharp_string + "/range/min", sharp_range_min_);
        nh_.getParam(default_laser_sharp_string + "/range/max", sharp_range_max_);
        nh_.getParam(default_laser_sharp_string + "/distance_center", sharp_distance_center_);
    } else {
        double laser_frequency = 40;
        nh_.setParam(default_laser_sharp_string + "/angle/min", -M_PI / 2);
        nh_.setParam(default_laser_sharp_string + "/angle/max", M_PI / 2);
        nh_.setParam(default_laser_sharp_string + "/angle/increment", (M_PI) / (NUMBER_INFRARED - 1));
        nh_.setParam(default_laser_sharp_string + "/time/increment", (1 / laser_frequency) / (NUMBER_INFRARED));
        nh_.setParam(default_laser_sharp_string + "/range/min", 0.04);
        nh_.setParam(default_laser_sharp_string + "/range/max", 0.40);
        nh_.setParam(default_laser_sharp_string + "/distance_center", 0.0);
    }
    if (nh_.hasParam(default_parameter_string)) {
        ROS_DEBUG("Sync parameter %s: ROS -> ROBOT", default_parameter_string.c_str());
        parameter_sensor_t parameter = getParameter();
        list_send->push_back(serial_->createDataPacket(PARAMETER_SENSOR, HASHMAP_NAVIGATION, (abstract_message_u*) & parameter));
    } else {
        ROS_DEBUG("Sync parameter %s: ROBOT -> ROS", default_parameter_string.c_str());
        list_send->push_back(serial_->createPacket(PARAMETER_SENSOR, REQUEST, HASHMAP_NAVIGATION));
    }
    //Request state enable sensor
    list_send->push_back(serial_->createPacket(ENABLE_SENSOR, REQUEST, HASHMAP_NAVIGATION));
}

void ROSSensorController::sensorPacket(const unsigned char& command, const abstract_message_u* packet) {
    ros_serial_bridge::Sensor sensor;
    sensor_msgs::Temperature temperature;
    switch (command) {
        case ENABLE_SENSOR:
            //            enable_sensor_ = packet->enable_sensor;
            enable = packet->enable_sensor;
            ROS_INFO("Enable state: %d", packet->enable_sensor);
            break;
        case INFRARED:
            sendLaserSharp(packet->infrared);
            break;
        case SENSOR:
            sensor.current = packet->sensor.current;
            sensor.temperature = packet->sensor.temperature;
            sensor.voltage = packet->sensor.voltage;
            pub_sensors.publish(sensor);
            //Temperature
            temperature.header.frame_id = base_link_string_;
            temperature.header.stamp = ros::Time::now();
            temperature.temperature = packet->sensor.temperature;
            temperature.variance = 0.5;
            pub_temperature.publish(temperature);
            break;
        case PARAMETER_SENSOR:
            nh_.setParam(default_parameter_string + "/sharp/exp", packet->parameter_sensor.exp_sharp);
            nh_.setParam(default_parameter_string + "/sharp/k", packet->parameter_sensor.gain_sharp);
            nh_.setParam(default_parameter_string + "/humidity/k", packet->parameter_sensor.gain_humidity);
            nh_.setParam(default_parameter_string + "/ali/current", packet->parameter_sensor.gain_current);
            nh_.setParam(default_parameter_string + "/ali/voltage", packet->parameter_sensor.gain_voltage);
            nh_.setParam(default_parameter_string + "/temperature/k", packet->parameter_sensor.gain_temperature);
            break;
    }
}

void ROSSensorController::sendLaserSharp(infrared_t infrared) {
    ros::Time scan_time = ros::Time::now();

    //Update information position laser sharp
    if (dynamic_update) {
        double x, y, z, theta;
        nh_.getParam("/tf/" + laser_sharp_position_string + "/x", x);
        nh_.getParam("/tf/" + laser_sharp_position_string + "/y", y);
        nh_.getParam("/tf/" + laser_sharp_position_string + "/z", z);
        nh_.getParam("/tf/" + laser_sharp_position_string + "/theta", theta);
        pose_laser_sharp = tf::Vector3(x, y, z);
        angle_laser_sharp = tf::createQuaternionFromYaw(theta);
    }

    broadcaster.sendTransform(
            tf::StampedTransform(
            tf::Transform(angle_laser_sharp, pose_laser_sharp),
            scan_time, base_link_string_, laser_sharp_string_));

    //Update information laser sharp
    if (dynamic_update) {
        nh_.getParam(default_laser_sharp_string + "/angle/min", sharp_angle_min_);
        nh_.getParam(default_laser_sharp_string + "/angle/max", sharp_angle_max_);
        nh_.getParam(default_laser_sharp_string + "/angle/increment", sharp_angle_increment_);
        nh_.getParam(default_laser_sharp_string + "/time/increment", sharp_time_increment_);
        nh_.getParam(default_laser_sharp_string + "/range/min", sharp_range_min_);
        nh_.getParam(default_laser_sharp_string + "/range/max", sharp_range_max_);
        nh_.getParam(default_laser_sharp_string + "/distance_center", sharp_distance_center_);
    }

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
        scan.ranges[i] = sharp_distance_center_ + infrared.infrared[i] / 100;
        scan.intensities[i] = infrared.infrared[i];
    }

    pub_laser_sharp.publish(scan);
}

parameter_sensor_t ROSSensorController::getParameter() {
    parameter_sensor_t parameter;
    double temp;
    nh_.getParam(default_parameter_string + "/sharp/exp", temp);
    parameter.exp_sharp = temp;
    nh_.getParam(default_parameter_string + "/sharp/k", temp);
    parameter.gain_sharp = temp;
    nh_.getParam(default_parameter_string + "/humidity/k", temp);
    parameter.gain_humidity = temp;
    nh_.getParam(default_parameter_string + "/ali/current", temp);
    parameter.gain_current = temp;
    nh_.getParam(default_parameter_string + "/ali/voltage", temp);
    parameter.gain_voltage = temp;
    nh_.getParam(default_parameter_string + "/temperature/k", temp);
    parameter.gain_temperature = temp;
    return parameter;
}

void ROSSensorController::enableCallback(const ros_serial_bridge::Enable::ConstPtr &msg) {
    enable_sensor_t enable = msg->enable;
    try {
        serial_->parserSendPacket(serial_->createDataPacket(ENABLE_SENSOR, HASHMAP_NAVIGATION, (abstract_message_u*) & enable), 3, boost::posix_time::millisec(200));
    } catch (exception &e) {
        ROS_ERROR("%s", e.what());
    }
}

bool ROSSensorController::parameterCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
    parameter_sensor_t parameter = getParameter();
    try {
        serial_->parserSendPacket(serial_->createDataPacket(PARAMETER_SENSOR, HASHMAP_NAVIGATION, (abstract_message_u*) & parameter), 3, boost::posix_time::millisec(200));
        return true;
    } catch (exception &e) {
        ROS_ERROR("%s", e.what());
    }
    return false;
}
