#include <teleop_key.h>
#include "std_msgs/String.h"

//int kfd = 0;
//struct termios cooked, raw;

std::string serial_bridge_string = "robot";
std::string command = "command";
std::string velocity = "velocity";
std::string enable = "enable";

std::string name_node = "drive_bridge";

/*void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}*/


int main(int argc, char** argv)
{
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

	TeleopKeybrd teleop(nh, serial_bridge_string, command, velocity, enable);

	//signal(SIGINT,quit);

	teleop.keyLoop();

  return(0);
}
