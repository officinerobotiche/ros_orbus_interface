/*
 * File:   unav_hwinterface.cpp
 * Author: Raffaello Bonghi
 *
 * Created on April 26, 2015, 12:00 PM
 */

#include <ros/ros.h>

using namespace std;

int main(int argc, char **argv) {

    ros::init(argc, argv, "ros_serial_bridge");

    string name_node = ros::this_node::getName();
    ROS_INFO("Started %s", name_node.c_str());

}
