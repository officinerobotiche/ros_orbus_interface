/* 
 * File:   serial_motion_node.cpp
 * Author: Raffaello Bonghi
 *
 * Created on June 7, 2013, 4:16 PM
 */

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "Serial.h"
#include "serial_controller/ROSControllerSerial.h"

#include <boost/thread.hpp>

const int rate = 10;
int arduino = 0;

std::string name_node = "serial_bridge_node";

/*
 * 
 */
int main(int argc, char** argv)
{
  //Init the serial_motion_node
  ros::init(argc, argv, name_node);
  if (argc != 2)
  {
    ROS_ERROR("need serial port name as argument");
    return -1;
  };

  //Name serial port
  std::string serial_port(argv[1]);
  
  ros::NodeHandle nh;
  std::string name = ros::this_node::getName();
  ROS_INFO("%s", name.c_str());
  //Serial* serial = new Serial("/dev/ttyUSB0", 115200);
  Serial* serial = new Serial(serial_port, 115200);
  if (arduino)
  {
    ROS_INFO("Wait to start Arduino (2sec) ... ");
    sleep(2);
    ROS_INFO("Serial Arduino started");
  }
  //Start ros serial controller
  RosControllerSerial* controller = new RosControllerSerial(name, nh, serial, rate);
  controller->loadParameter();

  ROS_INFO("start serial motion node");
  boost::thread* thr_stream_ = controller->run();

  thr_stream_->detach();
  
  ros::spin();


  return 0;
}
