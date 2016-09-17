/*
 * Copyright (C) 2016 Officine Robotiche
 * Author: Raffaello Bonghi
 * email:  raffaello@rnext.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU Lesser General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef UNAVHARDWARE_H
#define	UNAVHARDWARE_H
#include "ORBHardware.h"

#include <urdf/model.h>

#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/joint_command_interface.h"
#include "diagnostic/MotorTask.h"

#include <joint_limits_interface/joint_limits_interface.h>

#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

#include "configurator/MotorPIDConfigurator.h"
#include "configurator/MotorParamConfigurator.h"
#include "configurator/MotorEmergencyConfigurator.h"

#define NUM_MOTORS 2

class UNAVHardware : public ORBHardware {
public:
    UNAVHardware(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh, SerialController* serial, double frequency);
    virtual ~UNAVHardware();

    void updateJointsFromHardware();
    void writeCommandsToHardware(ros::Duration period);

protected:

    /// Initialize the diagnostic messages
    void initializeDiagnostics();

private:
    /// URDF information about robot
    boost::shared_ptr<urdf::ModelInterface> urdf_;
    /// Decode a motor command
    motor_command_map_t motor_command_;

    /// ROS Control interfaces
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::VelocityJointInterface velocity_joint_interface_;
    /// ROS joint limits interface
    joint_limits_interface::VelocityJointSoftLimitsInterface vel_limits_interface_;

    /// Register all control interface and joint limit interface
    void registerControlInterfaces();

    /// Setup all limits
    void setupLimits(hardware_interface::JointHandle joint_handle, std::string name, int i);

    void motorPacket(const unsigned char& command, const message_abstract_u* packet);
    void loadMotorParameter(std::vector<packet_information_t>* list_send);

    /**
    * Joint structure that is hooked to ros_control's InterfaceManager, to allow control via diff_drive_controller
    */
    struct Joint
    {
      // Diagnostics
      ros::Publisher diagnostic_publisher_;
      orbus_msgs::MotorStatus motor_status_msg_;
      MotorTask *motor_task_;
      // Configurator
      MotorPIDConfigurator *configurator_pid_velocity, *configurator_pid_effort, *configurator_pid_position;
      MotorParamConfigurator *configurator_param;
      MotorEmergencyConfigurator *configurator_emergency;
      // Actual state
      motor_state_t state;

      double position;
      double position_offset;
      double velocity;
      double effort;
      double velocity_command;

      Joint() : position(0), velocity(0), effort(0), velocity_command(0) { }
    } joints_[NUM_MOTORS];

};

#endif	/* UNAVHARDWARE_H */

