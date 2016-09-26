#ifndef UNAVCONTROLLER_H
#define UNAVCONTROLLER_H

#include <ros/ros.h>

#include <hardware_interface/robot_hw.h>

#include <urdf/model.h>

#include "hardware/Motor.h"
#include "hardware/GenericController.h"

namespace ORController
{

class uNavController : public GenericController, public hardware_interface::RobotHW
{
public:
    uNavController(const ros::NodeHandle &nh, orbus::serial_controller *serial);

    void updateJointsFromHardware();
    void writeCommandsToHardware(ros::Duration period);

private:

    void registerControlInterfaces();

    void allMotorsFrame(unsigned char option, unsigned char type, unsigned char command, message_abstract_u message);

private:
    /// URDF information about robot
    shared_ptr<urdf::ModelInterface> urdf;

    // List of motors
    vector<Motor> list_motor;

    /// ROS Control interfaces
    hardware_interface::JointStateInterface joint_state_interface;
    hardware_interface::VelocityJointInterface velocity_joint_interface;
};

}

#endif // UNAVCONTROLLER_H
