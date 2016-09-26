#ifndef MOTOR_H
#define MOTOR_H

#include <ros/ros.h>
#include <urdf/model.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <joint_limits_interface/joint_limits_interface.h>

#include "hardware/serial_controller.h"

using namespace std;

namespace ORInterface
{

class Motor : public diagnostic_updater::DiagnosticTask
{
public:
    explicit Motor(const ros::NodeHandle &nh, orbus::serial_controller *serial, unsigned int number);

    void run(diagnostic_updater::DiagnosticStatusWrapper &stat);

    void registerControlInterfaces(hardware_interface::JointStateInterface joint_state_interface,
                                   hardware_interface::VelocityJointInterface velocity_joint_interface,
                                   shared_ptr<urdf::ModelInterface> urdf);

    void motorFrame(unsigned char option, unsigned char type, unsigned char command, message_abstract_u message);

private:

    void setupLimits(hardware_interface::JointHandle joint_handle, shared_ptr<urdf::ModelInterface> urdf);

private:
    //Initialization object
    //NameSpace for bridge controller
    ros::NodeHandle mNh;
    // Serial controller communication
    orbus::serial_controller *mSerial;
    // Name of the motor
    string mName;
    // State of the motor
    double position;
    double position_offset;
    double velocity;
    double effort;
    double velocity_command;

    /// ROS joint limits interface
    joint_limits_interface::VelocityJointSoftLimitsInterface vel_limits_interface;

    // Publisher diagnostic information
    ros::Publisher diagnostic_publisher;

    // Number message
    motor_command_map_t command;
};

}

#endif // MOTOR_H
