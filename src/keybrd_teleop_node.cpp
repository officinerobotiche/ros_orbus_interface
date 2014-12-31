#include <teleop_key.h>
#include "std_msgs/String.h"

std::string serial_bridge_string = "robot";
std::string command = "command";
std::string velocity = "velocity";
std::string enable = "enable";

std::string name_node = "drive_bridge";

int main(int argc, char** argv)
{
	ros::init(argc, argv, name_node);
	ros::NodeHandle nh;

	//Load configuration

    if (nh.hasParam("/info/robot_name")) {
        nh.getParam("/info/robot_name", serial_bridge_string);
    } else {
        nh.setParam("/info/robot_name", serial_bridge_string);
    }

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

	TeleopKeybrd teleop(nh, serial_bridge_string, command, velocity, enable);

	teleop.keyLoop();

  return(0);
}
