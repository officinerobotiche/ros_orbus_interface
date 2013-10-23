/* 
 * File:   spacenav_teleop.cpp
 * Author: raffaello
 *
 * Created on 10 September 2013, 13:22
 */

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <signal.h>
#include <cstdlib>
#include "drive_bridge/Keyboard.h"

using namespace std;
Keyboard* keyboard;

std::string robot = "robot";

void quit(int sig)
{
  keyboard->quit(sig);
  exit(0);
}

/*
 * 
 */
int main(int argc, char** argv)
{

  ros::init(argc, argv, "drive_bridge");
  ros::NodeHandle nh;
  keyboard = new Keyboard(nh, robot);
  signal(SIGINT, quit);
  
  ROS_INFO("Wait 2sec...");
  sleep(2);
  ROS_INFO("Ready!");
  ros::spinOnce();

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the turtle.");
  
  keyboard->read_keyboard();

  return 0;
}

