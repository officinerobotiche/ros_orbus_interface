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
: ROSController(name_node, nh, serial) {
    string param_name_board = "Navigation Board";
    if (name_board.compare(name_board) == 0) {
        nh_.setParam(name_node + "/name_board", name_board);
    } else {
        throw (controller_exception("Other board: " + name_board));
    }

    serial->addCallback(&ROSTempSensorController::sensorPacket, this, HASHMAP_NAVIGATION);

    //Open Publisher
    pub_laser_sharp = nh_.advertise<sensor_msgs::LaserScan>(name_node + "/" + default_laser_sharp_string, NUMBER_PUB,
            boost::bind(&ROSController::connectCallback, this, _1));
    pub_sensors = nh_.advertise<serial_bridge::Sensor>(name_node + "/" + default_sensor_string, NUMBER_PUB,
            boost::bind(&ROSController::connectCallback, this, _1));

}

ROSTempSensorController::~ROSTempSensorController() {
    serial_->clearCallback(HASHMAP_NAVIGATION);
}

void ROSTempSensorController::sensorPacket(const unsigned char& command, const abstract_packet_t* packet) {
    serial_bridge::Sensor sensor;
    switch (command) {
        case ENABLE_SENSOR:
            //            enable_sensor_ = packet->enable_sensor;
            ROS_INFO("Enable state: %d", packet->enable_sensor);
            break;
        case INFRARED:
            //            sendLaserSharp(packet->infrared);
            break;
        case SENSOR:
            sensor.current = packet->sensor.current;
            sensor.temperature = packet->sensor.temperature;
            sensor.voltage = packet->sensor.voltage;
            pub_sensors.publish(sensor);
            break;
    }
}