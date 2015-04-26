#ifndef UNAV_HARDWARE_H
#define UNAV_HARDWARE_H

#include <ros/ros.h>
#include "hardware/orb_hardware.h"

class unav_hardware : public orb_hardware::ORBHardware
{
public:
    unav_hardware(const ros::NodeHandle& nh, ros::NodeHandle private_nh, ParserPacket* serial, double target_control_freq);

    void updateJointsFromHardware();

    void writeCommandsToHardware();

private:

    void registerControlInterfaces();

    void motionPacket(const unsigned char& command, const abstract_message_u* packet);

    // ROS Control interfaces
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::VelocityJointInterface velocity_joint_interface_;

    /**
    * Joint structure that is hooked to ros_control's InterfaceManager, to allow control via diff_drive_controller
    */
    struct Joint
    {
      double position;
      double position_offset;
      double velocity;
      double effort;
      double velocity_command;

      Joint() : position(0), velocity(0), effort(0), velocity_command(0) { }
    } joints_[2];
};

#endif // UNAV_HARDWARE_H
