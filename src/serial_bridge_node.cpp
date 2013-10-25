/* 
 * File:   serial_motion_node.cpp
 * Author: Raffaello Bonghi
 *
 * Created on June 7, 2013, 4:16 PM
 */

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "Serial.h"
#include "serial_controller/ROSMotionController.h"
#include "serial_controller/ROSSensorController.h"

#include <iostream>
#include <exception>
#include <boost/thread.hpp>

const int rate = 10;
int arduino = 0;

std::string name_node = "serial_bridge_node";
std::string name_motion_control = "Motion Control";
std::string name_navigation_board = "Navigation Board";

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
  int baud_rate = 115200;
  if (nh.hasParam(name_node + "/baud_rate"))
  {
    nh.getParam(name_node + "/baud_rate", baud_rate);
  }
  else
  {
    nh.setParam(name_node + "/baud_rate", baud_rate);
  }
  Serial* serial;
  try
  {
    serial = new Serial(serial_port, baud_rate);
    ROS_INFO("Open Serial %s:%d", serial_port.c_str(), baud_rate);
  }
  catch (std::exception& e)
  {
    ROS_ERROR("An exception occurred. Exception: %s", e.what());
    return -1;
  }

  if (arduino)
  {
    ROS_INFO("Wait to start Arduino (2sec) ... ");
    sleep(2);
    ROS_INFO("Serial Arduino started");
  }
  //Start ros serial controller
  AbstractROSController * controller;
  std::string name_board;
  //TODO create object to contact board
  if (nh.hasParam(name_node + "/board"))
  {
    nh.getParam(name_node + "/board", name_board);
  }
  else
    name_board = name_motion_control;
  ROS_INFO("Manual set board: %s", name_board.c_str());
  if (name_board.compare(name_motion_control) == 0)
  {
    controller = new ROSMotionController(name, nh, serial, rate);
  }
  else if (name_board.compare(name_navigation_board) == 0)
  {
    controller = new ROSSensorController(name, nh, serial);
  }
  else
  {
    ROS_ERROR("Unknown board! name: %s", name_board.c_str());
    return -1;
  }
  //Start controller
  controller->loadParameter();
  if (name_board.compare(name_motion_control) == 0)
  {
    boost::thread* thr_stream_ = ((ROSMotionController*)controller)->run();
    thr_stream_->detach();
  }
  ROS_INFO("start %s", name.c_str());

  ros::spin();

  return 0;
}
