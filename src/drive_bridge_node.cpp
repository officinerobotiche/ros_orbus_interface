/* 
 * File:   spacenav_teleop.cpp
 * Author: raffaello
 *
 * Created on 10 September 2013, 13:22
 */

#include <ros/ros.h>
#include "std_msgs/String.h"

#include <signal.h>
#include <cstdlib>
#include "drive_bridge/Keyboard.h"

using namespace std;
Keyboard* keyboard;

std::string serial_bridge_string = "robot";
std::string command = "command";
std::string velocity = "velocity";
std::string enable = "enable";

std::string name_node = "drive_bridge";

void quit(int sig) {
    keyboard->quit(sig);
    exit(0);
}

/*
 * 
 */
int main(int argc, char** argv) {

    ros::init(argc, argv, name_node);
    ros::NodeHandle nh;

    //Load configuration
    if (nh.hasParam(name_node + "/serial_bridge")) {
        nh.getParam(name_node + "/serial_bridge", serial_bridge_string);
    } else {
        nh.setParam(name_node + "/serial_bridge", serial_bridge_string);
    }
    if (nh.hasParam(name_node + "/command")) {
        nh.getParam(name_node + "/command", command);
    } else {
        nh.setParam(name_node + "/command", command);
    }
    if (nh.hasParam(name_node + "/velocity")) {
        nh.getParam(name_node + "/velocity", velocity);
    } else {
        nh.setParam(name_node + "/velocity", velocity);
    }
    if (nh.hasParam(name_node + "/enable")) {
        nh.getParam(name_node + "/enable", enable);
    } else {
        nh.setParam(name_node + "/enable", enable);
    }
    
    //Init keyboard control
    keyboard = new Keyboard(nh, serial_bridge_string, command, velocity, enable);
    signal(SIGINT, quit);

    //  ROS_INFO("Wait 2sec...");
    //  sleep(2);
    //  ROS_INFO("Ready!");
    //  ros::spinOnce();

    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use arrow keys to move the robot.");

    //Start keyboard read
    keyboard->read_keyboard();

    return 0;
}
