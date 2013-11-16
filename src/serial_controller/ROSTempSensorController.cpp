/* 
 * File:   ROSTempSensorController.cpp
 * Author: raffaello
 * 
 * Created on 16 November 2013, 11:49
 */

#include "serial_controller/ROSTempSensorController.h"
#include <sensor_msgs/LaserScan.h>
#include <serial_bridge/Sensor.h>

#define NUMBER_PUB 10

using namespace std;

ROSTempSensorController::ROSTempSensorController(std::string name_node, const ros::NodeHandle& nh, ParserPacket* serial)
: ROSController(name_node, nh, serial), dynamic_update(false) {

    string param_name_board = "Navigation Board";
    if (name_board.compare(param_name_board) == 0) {
        nh_.setParam(name_node + "/name_board", name_board);
    } else {
        throw (controller_exception("Other board: " + name_board));
    }

    serial->addCallback(&ROSTempSensorController::sensorPacket, this, HASHMAP_NAVIGATION);
    //    addVectorPacketRequest(&ROSMotionController::updatePacket, this);
    addParameterPacketRequest(&ROSTempSensorController::addParameter, this);
    //    addTimerEvent(&ROSMotionController::timerEvent, this);

    //Open Publisher
    pub_laser_sharp = nh_.advertise<sensor_msgs::LaserScan>(name_node + "/" + default_laser_sharp_string, NUMBER_PUB,
            boost::bind(&ROSController::connectCallback, this, _1));
    pub_sensors = nh_.advertise<serial_bridge::Sensor>(name_node + "/" + default_sensor_string, NUMBER_PUB,
            boost::bind(&ROSController::connectCallback, this, _1));
    //Open Subscriber
    //-Command
    sub_enable = nh_.subscribe(name_node + "/" + command_string + "/" + enable_sensors, 1, &ROSTempSensorController::enableCallback, this);

    //Open Service
    srv_parameter = nh_.advertiseService(name_node + "/" + default_parameter_string, &ROSTempSensorController::parameterCallback, this);


}

ROSTempSensorController::~ROSTempSensorController() {
    serial_->clearCallback(HASHMAP_NAVIGATION);
    //    clearVectorPacketRequest();
    clearParameterPacketRequest();
    //    clearTimerEvent();
}

void ROSTempSensorController::addParameter(std::vector<information_packet_t>* list_send) {
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
    if (nh_.hasParam(name_node_ + "/tf/" + laser_sharp_position_string + "/dynamic_update")) {
        nh_.getParam(name_node_ + "/tf/" + laser_sharp_position_string + "/dynamic_update", dynamic_update);
    } else {
        nh_.setParam(name_node_ + "/tf/" + laser_sharp_position_string + "/dynamic_update", false);
    }
    if (nh_.hasParam(name_node_ + "/tf/" + laser_sharp_position_string)) {
        double x, y, z, theta;
        nh_.getParam(name_node_ + "/tf/" + laser_sharp_position_string + "/x", x);
        nh_.getParam(name_node_ + "/tf/" + laser_sharp_position_string + "/y", y);
        nh_.getParam(name_node_ + "/tf/" + laser_sharp_position_string + "/z", z);
        nh_.getParam(name_node_ + "/tf/" + laser_sharp_position_string + "/theta", theta);
        pose_laser_sharp = tf::Vector3(x, y, z);
        angle_laser_sharp = tf::createQuaternionFromYaw(theta);
    } else {
        nh_.setParam(name_node_ + "/tf/" + laser_sharp_position_string + "/x", 0.0);
        nh_.setParam(name_node_ + "/tf/" + laser_sharp_position_string + "/y", 0.0);
        nh_.setParam(name_node_ + "/tf/" + laser_sharp_position_string + "/z", 0.0);
        nh_.setParam(name_node_ + "/tf/" + laser_sharp_position_string + "/theta", 0.0);
        pose_laser_sharp = tf::Vector3(0.0, 0.0, 0.0);
        angle_laser_sharp = tf::Quaternion(0, 0, 0, 1);
    }
    //Set laser scan
    if (nh_.hasParam(name_node_ + "/" + default_laser_sharp_string)) {
        nh_.getParam(name_node_ + "/" + default_laser_sharp_string + "/angle/min", sharp_angle_min_);
        nh_.getParam(name_node_ + "/" + default_laser_sharp_string + "/angle/max", sharp_angle_max_);
        nh_.getParam(name_node_ + "/" + default_laser_sharp_string + "/angle/increment", sharp_angle_increment_);
        nh_.getParam(name_node_ + "/" + default_laser_sharp_string + "/time/increment", sharp_time_increment_);
        nh_.getParam(name_node_ + "/" + default_laser_sharp_string + "/range/min", sharp_range_min_);
        nh_.getParam(name_node_ + "/" + default_laser_sharp_string + "/range/max", sharp_range_max_);
        nh_.getParam(name_node_ + "/" + default_laser_sharp_string + "/distance_center", sharp_distance_center_);
    } else {
        double laser_frequency = 40;
        nh_.setParam(name_node_ + "/" + default_laser_sharp_string + "/angle/min", -M_PI / 2);
        nh_.setParam(name_node_ + "/" + default_laser_sharp_string + "/angle/max", M_PI / 2);
        nh_.setParam(name_node_ + "/" + default_laser_sharp_string + "/angle/increment", (M_PI) / (NUMBER_INFRARED));
        nh_.setParam(name_node_ + "/" + default_laser_sharp_string + "/time/increment", (1 / laser_frequency) / (NUMBER_INFRARED));
        nh_.setParam(name_node_ + "/" + default_laser_sharp_string + "/range/min", 0.04);
        nh_.setParam(name_node_ + "/" + default_laser_sharp_string + "/range/max", 0.40);
        nh_.setParam(name_node_ + "/" + default_laser_sharp_string + "/distance_center", 0.0);
    }
    if (nh_.hasParam(name_node_ + "/" + default_parameter_string)) {
        ROS_INFO("Sync parameter %s: ROS -> ROBOT", default_parameter_string.c_str());
        parameter_sensor_t parameter = getParameter();
        list_send->push_back(serial_->createDataPacket(PARAMETER_SENSOR, HASHMAP_NAVIGATION, (abstract_packet_t*) & parameter));
    } else {
        ROS_INFO("Sync parameter %s: ROBOT -> ROS", default_parameter_string.c_str());
        list_send->push_back(serial_->createPacket(PARAMETER_SENSOR, REQUEST, HASHMAP_NAVIGATION));
    }
}

void ROSTempSensorController::sensorPacket(const unsigned char& command, const abstract_packet_t* packet) {
    serial_bridge::Sensor sensor;
    switch (command) {
        case ENABLE_SENSOR:
            //            enable_sensor_ = packet->enable_sensor;
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
            break;
        case PARAMETER_SENSOR:
            nh_.setParam("/" + name_node_ + "/" + default_parameter_string + "/sharp/exp", packet->parameter_sensor.exp_sharp);
            nh_.setParam("/" + name_node_ + "/" + default_parameter_string + "/sharp/k", packet->parameter_sensor.gain_sharp);
            nh_.setParam("/" + name_node_ + "/" + default_parameter_string + "/humidity/k", packet->parameter_sensor.gain_humidity);
            nh_.setParam("/" + name_node_ + "/" + default_parameter_string + "/ali/current", packet->parameter_sensor.gain_current);
            nh_.setParam("/" + name_node_ + "/" + default_parameter_string + "/ali/voltage", packet->parameter_sensor.gain_voltage);
            nh_.setParam("/" + name_node_ + "/" + default_parameter_string + "/temperature/k", packet->parameter_sensor.gain_temperature);
            break;
    }
}

void ROSTempSensorController::sendLaserSharp(infrared_t infrared) {
    ros::Time scan_time = ros::Time::now();

    //Update information position laser sharp
    if (dynamic_update) {
        double x, y, z, theta;
        nh_.getParam(name_node_ + "/tf/" + laser_sharp_position_string + "/x", x);
        nh_.getParam(name_node_ + "/tf/" + laser_sharp_position_string + "/y", y);
        nh_.getParam(name_node_ + "/tf/" + laser_sharp_position_string + "/z", z);
        nh_.getParam(name_node_ + "/tf/" + laser_sharp_position_string + "/theta", theta);
        pose_laser_sharp = tf::Vector3(x, y, z);
        angle_laser_sharp = tf::createQuaternionFromYaw(theta);
    }

    broadcaster.sendTransform(
            tf::StampedTransform(
            tf::Transform(angle_laser_sharp, pose_laser_sharp),
            scan_time, base_link_string_, laser_sharp_string_));

    //Update information laser sharp
    if (dynamic_update) {
        nh_.getParam(name_node_ + "/" + default_laser_sharp_string + "/angle/min", sharp_angle_min_);
        nh_.getParam(name_node_ + "/" + default_laser_sharp_string + "/angle/max", sharp_angle_max_);
        nh_.getParam(name_node_ + "/" + default_laser_sharp_string + "/angle/increment", sharp_angle_increment_);
        nh_.getParam(name_node_ + "/" + default_laser_sharp_string + "/time/increment", sharp_time_increment_);
        nh_.getParam(name_node_ + "/" + default_laser_sharp_string + "/range/min", sharp_range_min_);
        nh_.getParam(name_node_ + "/" + default_laser_sharp_string + "/range/max", sharp_range_max_);
        nh_.getParam(name_node_ + "/" + default_laser_sharp_string + "/distance_center", sharp_distance_center_);
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

parameter_sensor_t ROSTempSensorController::getParameter() {
    parameter_sensor_t parameter;
    double temp;
    nh_.getParam(name_node_ + "/" + default_parameter_string + "/sharp/exp", temp);
    parameter.exp_sharp = temp;
    nh_.getParam(name_node_ + "/" + default_parameter_string + "/sharp/k", temp);
    parameter.gain_sharp = temp;
    nh_.getParam(name_node_ + "/" + default_parameter_string + "/humidity/k", temp);
    parameter.gain_humidity = temp;
    nh_.getParam(name_node_ + "/" + default_parameter_string + "/ali/current", temp);
    parameter.gain_current = temp;
    nh_.getParam(name_node_ + "/" + default_parameter_string + "/ali/voltage", temp);
    parameter.gain_voltage = temp;
    nh_.getParam(name_node_ + "/" + default_parameter_string + "/temperature/k", temp);
    parameter.gain_temperature = temp;
    return parameter;
}

void ROSTempSensorController::enableCallback(const serial_bridge::Enable::ConstPtr &msg) {
    enable_sensor_t enable = msg->enable;
    try {
        serial_->parserSendPacket(serial_->createDataPacket(ENABLE_SENSOR, HASHMAP_NAVIGATION, (abstract_packet_t*) & enable), 3, boost::posix_time::millisec(200));
    } catch (exception &e) {
        ROS_ERROR("%s", e.what());
    }
}

bool ROSTempSensorController::parameterCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
    parameter_sensor_t parameter = getParameter();
    try {
        serial_->parserSendPacket(serial_->createDataPacket(PARAMETER_SENSOR, HASHMAP_NAVIGATION, (abstract_packet_t*) & parameter), 3, boost::posix_time::millisec(200));
        return true;
    } catch (exception &e) {
        ROS_ERROR("%s", e.what());
    }
    return false;
}