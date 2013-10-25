/* 
 * File:   ROSSensorController.cpp
 * Author: raffaello
 * 
 * Created on 25 October 2013, 13:27
 */

#include "serial_controller/ROSSensorController.h"

ROSSensorController::ROSSensorController(std::string name_node, const ros::NodeHandle& nh, Serial* serial)
  : nh_(nh)
{
  name_node_ = name_node; // Initialize node name
  this->serial_ = serial; // Initialize serial port
}

ROSSensorController::ROSSensorController(const ROSSensorController& orig)
{
}

ROSSensorController::~ROSSensorController()
{
}

void ROSSensorController::loadParameter()
{

}