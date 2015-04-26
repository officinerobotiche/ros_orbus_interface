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


#include "hardware/unav_hardware.h"
#include <boost/assign/list_of.hpp>

unav_hardware::unav_hardware(const ros::NodeHandle& nh, ros::NodeHandle private_nh, ParserPacket* serial, double target_control_freq)
    : orb_hardware::ORBHardware(nh, private_nh, serial, target_control_freq)
{
    serial->addCallback(&unav_hardware::motionPacket, this, HASHMAP_MOTION);
    //registerControlInterfaces();
}

/**
* @brief unav_hardware::registerControlInterfaces Register interfaces with the RobotHW interface manager, allowing ros_control operation
*/
void unav_hardware::registerControlInterfaces()
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
* @brief unav_hardware::updateJointsFromHardware Pull latest speed and travel measurements from MCU, and store in joint structure for ros_control
*/
void unav_hardware::updateJointsFromHardware()
{
    ROS_INFO("Update Joints");
}

/**
* @brief unav_hardware::writeCommandsToHardware Get latest velocity commands from ros_control via joint structure, and send to MCU
*/
void unav_hardware::writeCommandsToHardware()
{
    ROS_INFO("Write command");
}


void unav_hardware::motionPacket(const unsigned char& command, const abstract_message_u* packet) {
    switch (command) {
        case CONSTRAINT:
        //TODO
            //nh_.setParam(joint_string + "/constraint/" + right_string, packet->constraint.max_right);
            //nh_.setParam(joint_string + "/constraint/" + left_string, packet->constraint.max_left);
            break;
        case PARAMETER_UNICYCLE:
        //TODO
            //nh_.setParam("structure/" + wheelbase_string, packet->parameter_unicycle.wheelbase);
            //nh_.setParam("structure/" + radius_string + "/" + right_string, packet->parameter_unicycle.radius_r);
            //nh_.setParam("structure/" + radius_string + "/" + left_string, packet->parameter_unicycle.radius_l);
            //nh_.setParam("odo_mis_step", packet->parameter_unicycle.sp_min);
            break;
        case PARAMETER_MOTOR_L:
        //TODO
            //nh_.setParam(joint_string + "/" + left_string + "/k_vel", packet->parameter_motor.k_vel);
            //nh_.setParam(joint_string + "/" + left_string + "/k_ang", packet->parameter_motor.k_ang);
            //nh_.setParam(joint_string + "/" + left_string + "/encoder_swap", packet->parameter_motor.versus);
            //nh_.setParam(joint_string + "/" + left_string + "/default_enable", packet->parameter_motor.enable_set);
            break;
        case PARAMETER_MOTOR_R:
        //TODO
            //nh_.setParam(joint_string + "/" + right_string + "/k_vel", packet->parameter_motor.k_vel);
            //nh_.setParam(joint_string + "/" + right_string + "/k_ang", packet->parameter_motor.k_ang);
            //nh_.setParam(joint_string + "/" + right_string + "/versus", packet->parameter_motor.versus);
            //nh_.setParam(joint_string + "/" + right_string + "/default_enable", packet->parameter_motor.enable_set);
            break;
        case EMERGENCY:
        //TODO
            //nh_.setParam(emergency_string + "/bridge_off", packet->emergency.bridge_off);
            //nh_.setParam(emergency_string + "/slope_time", packet->emergency.slope_time);
            //nh_.setParam(emergency_string + "/timeout", ((double) packet->emergency.timeout) / 1000.0);
        break;
        case PID_CONTROL_L:
        //TODO
            //name_pid = "pid/" + left_string + "/";
            //nh_.setParam(name_pid + "P", packet->pid.kp);
            //nh_.setParam(name_pid + "I", packet->pid.ki);
            //nh_.setParam(name_pid + "D", packet->pid.kd);
            break;
        case PID_CONTROL_R:
        //TODO
            //name_pid = "pid/" + right_string + "/";
            //nh_.setParam(name_pid + "P", packet->pid.kp);
            //nh_.setParam(name_pid + "I", packet->pid.ki);
            //nh_.setParam(name_pid + "D", packet->pid.kd);
            break;
        case VEL_MOTOR_MIS_L:

            break;
        case VEL_MOTOR_MIS_R:

            break;
        case MOTOR_L:
        //TODO
            //motor_left.reference = ((double) packet->motor.refer_vel) / 1000;
            //motor_left.control = ((double) packet->motor.control_vel) * (1000.0 / INT16_MAX);
            //motor_left.measure = ((double) packet->motor.measure_vel) / 1000;
            //motor_left.current = ((double) packet->motor.current) / 1000;
            //pub_motor_left.publish(motor_left);
            break;
        case MOTOR_R:
        //TODO
            //motor_right.reference = ((double) packet->motor.refer_vel) / 1000;
            //motor_right.control = ((double) packet->motor.control_vel) * (1000.0 / INT16_MAX);
            //motor_right.measure = ((double) packet->motor.measure_vel) / 1000;
            //motor_right.current = ((double) packet->motor.current) / 1000;
            //pub_motor_right.publish(motor_right);
            break;
        case COORDINATE:
        //TODO
            //pose.x = packet->coordinate.x;
            //pose.y = packet->coordinate.y;
            //pose.theta = packet->coordinate.theta;
            //pose.space = packet->coordinate.space;
            //pub_pose.publish(pose);
            break;
        case VELOCITY:
        //TODO
            //twist.linear.x = packet->velocity.v;
            //twist.angular.z = packet->velocity.w;
            //pub_twist.publish(twist);
            break;
        case VELOCITY_MIS:
            //meas_velocity = packet->velocity;
            break;
        case ENABLE:
        //TODO
            //enable_motors.enable = packet->enable;
            //pub_enable.publish(enable_motors);
            break;
    }
}
