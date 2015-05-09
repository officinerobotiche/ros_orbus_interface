/*
 * File:   UNAVHardware.h
 * Author: raffaello
 *
 * Created on 15 November 2013, 18:34
 */

#ifndef UNAVHARDWARE_H
#define	UNAVHARDWARE_H

#include "ORBHardware.h"

#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/joint_command_interface.h"
#include <joint_limits_interface/joint_limits_interface.h>

#include "configurator/MotorPIDConfigurator.h"
#include "configurator/MotorParamConfigurator.h"
#include "configurator/MotorEmergencyConfigurator.h"

#define NUM_MOTORS 2

const std::string paramenter_unicycle_string = "unicycle";
const std::string wheelbase_string = "wheelbase";
const std::string radius_string = "radius";

class UNAVHardware : public ORBHardware {
public:
    UNAVHardware(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh, ParserPacket* serial);
    virtual ~UNAVHardware();

    void updateJointsFromHardware();
    void writeCommandsToHardware(ros::Duration period);

private:

    //Decode a motor command
    motor_command_map_t motor_command_;
    //List to send messages to serial
    std::vector<packet_information_t> list_send_;

    // ROS Control interfaces
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::VelocityJointInterface velocity_joint_interface_;

    joint_limits_interface::VelocityJointSoftLimitsInterface vel_limits_interface_;
    //Service
    //ros::ServiceServer srv_pid, srv_parameter, srv_constraint, srv_emergency;

    void registerControlInterfaces();

//    bool pidServiceCallback(ros_serial_bridge::Update::Request &req, ros_serial_bridge::Update::Response&);
//    bool parameterServiceCallback(ros_serial_bridge::Update::Request &req, ros_serial_bridge::Update::Response&);
//    bool constraintServiceCallback(ros_serial_bridge::Update::Request &req, ros_serial_bridge::Update::Response&);
//    bool emergencyServiceCallback(ros_serial_bridge::Update::Request &req, ros_serial_bridge::Update::Response&);

    void motionPacket(const unsigned char& command, const message_abstract_u* packet);
    void motorPacket(const unsigned char& command, const message_abstract_u* packet);
    void addParameter(std::vector<packet_information_t>* list_send);

    motor_t get_constraint(std::string name);
    motor_emergency_t get_emergency(std::string name);

    /**
    * Joint structure that is hooked to ros_control's InterfaceManager, to allow control via diff_drive_controller
    */
    struct Joint
    {
      MotorPIDConfigurator *configurator_pid;
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

