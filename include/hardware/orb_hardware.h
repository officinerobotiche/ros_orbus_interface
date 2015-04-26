#ifndef ORB_HARDWARE_H
#define ORB_HARDWARE_H

#include <ros/ros.h>
#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/robot_hw.h"

namespace orb_hardware
{
    /**
    * Class representing Husky hardware, allows for ros_control to modify internal state via joint interfaces
    */
    class ORBHardware : public hardware_interface::RobotHW
    {
    public:
        ORBHardware(ros::NodeHandle nh, ros::NodeHandle private_nh, double target_control_freq);

        void updateJointsFromHardware();

        void writeCommandsToHardware();

        void updateDiagnostics();

        void reportLoopDuration(const ros::Duration &duration);

    private:

        void registerControlInterfaces();

        ros::NodeHandle nh_, private_nh_;

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
}



#endif // ORB_HARDWARE_H
