/**
*
*  \author     Raffaello Bonghi <raffaello.bonghi@officinerobotiche.it>
*  \copyright  Copyright (c) 2014-2015, Officine Robotiche, Inc.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     * Neither the name of Clearpath Robotics, Inc. nor the
*       names of its contributors may be used to endorse or promote products
*       derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* Please send comments, questions, or patches to code@clearpathrobotics.com
*
*/

#include "hardware/orb_hardware.h"
#include <boost/assign/list_of.hpp>

namespace
{
  const uint8_t LEFT = 0, RIGHT = 1;
}

namespace orb_hardware
{
    /**
    * Initialize general ORB hardware
    */
    ORBHardware::ORBHardware(ros::NodeHandle nh, ros::NodeHandle private_nh, double target_control_freq)
        : nh_(nh),
          private_nh_(private_nh)
    {
        ROS_INFO("Test");
        registerControlInterfaces();
    }

    /**
    * Register interfaces with the RobotHW interface manager, allowing ros_control operation
    */
    void ORBHardware::registerControlInterfaces()
    {
      ros::V_string joint_names = boost::assign::list_of("left")("right");
      for (unsigned int i = 0; i < joint_names.size(); i++)
      {
        hardware_interface::JointStateHandle joint_state_handle(joint_names[i],
                                                                &joints_[i].position, &joints_[i].velocity, &joints_[i].effort);
        joint_state_interface_.registerHandle(joint_state_handle);

        hardware_interface::JointHandle joint_handle(
            joint_state_handle, &joints_[i].velocity_command);
        velocity_joint_interface_.registerHandle(joint_handle);
      }
      registerInterface(&joint_state_interface_);
      registerInterface(&velocity_joint_interface_);
    }

    /**
    * Pull latest speed and travel measurements from MCU, and store in joint structure for ros_control
    */
    void ORBHardware::updateJointsFromHardware()
    {
        ROS_INFO("Update Joints");
    }

    /**
    * Get latest velocity commands from ros_control via joint structure, and send to MCU
    */
    void ORBHardware::writeCommandsToHardware()
    {
        ROS_INFO("Write command");
    }

    /**
    * External hook to trigger diagnostic update
    */
    void ORBHardware::updateDiagnostics()
    {
        ROS_INFO("UpdateDiagnostics");
    }

    /**
    * Update diagnostics with control loop timing information
    */
    void ORBHardware::reportLoopDuration(const ros::Duration &duration)
    {
        ROS_INFO("Report LOOP");
    }
}

