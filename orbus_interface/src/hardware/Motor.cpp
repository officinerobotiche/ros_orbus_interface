
#include "hardware/Motor.h"

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>

#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

namespace ORController
{

Motor::Motor(const ros::NodeHandle& nh, orbus::serial_controller *serial, unsigned int number)
    : DiagnosticTask("motor_" + to_string(number) + "_status")
    , mNh(nh)
    , mSerial(serial)
{
    command.bitset.motor = number;
    mName = "motor_" + to_string(number);

}

void Motor::registerControlInterfaces(hardware_interface::JointStateInterface joint_state_interface, hardware_interface::VelocityJointInterface velocity_joint_interface, shared_ptr<urdf::ModelInterface> urdf)
{
    /// Joint hardware interface
    hardware_interface::JointStateHandle joint_state_handle(mName, &position, &velocity, &effort);

    joint_state_interface.registerHandle(joint_state_handle);

    /// Differential drive interface
    hardware_interface::JointHandle joint_handle(joint_state_handle, &velocity_command);
    velocity_joint_interface.registerHandle(joint_handle);

    setupLimits(joint_handle, urdf);
}

void Motor::setupLimits(hardware_interface::JointHandle joint_handle, shared_ptr<urdf::ModelInterface> urdf)
{
    /// Add a velocity joint limits infomations
    /// Populate with any of the methods presented in the previous example...
    joint_limits_interface::JointLimits limits;
    joint_limits_interface::SoftJointLimits soft_limits;

    // Manual value setting
    limits.has_velocity_limits = true;
    limits.max_velocity = 5.0;

    // Populate (soft) joint limits from URDF
    // Limits specified in URDF overwrite existing values in 'limits' and 'soft_limits'
    // Limits not specified in URDF preserve their existing values
    if(urdf != NULL) {
        boost::shared_ptr<const urdf::Joint> urdf_joint = urdf->getJoint(mName);
        const bool urdf_limits_ok = getJointLimits(urdf_joint, limits);
        const bool urdf_soft_limits_ok = getSoftJointLimits(urdf_joint, soft_limits);
        if(urdf_limits_ok) {
            ROS_INFO_STREAM("LOAD " << mName << " limits from URDF: " << limits.max_velocity << " rad/s");
        }
        if(urdf_soft_limits_ok) {
            ROS_INFO_STREAM("LOAD " << mName << " soft limits from URDF: " << limits.max_velocity << " rad/s");
        }
    }

    // Populate (soft) joint limits from the ros parameter server
    // Limits specified in the parameter server overwrite existing values in 'limits' and 'soft_limits'
    // Limits not specified in the parameter server preserve their existing values
    const bool rosparam_limits_ok = getJointLimits(mName, mNh, limits);
    if(rosparam_limits_ok) {
        ROS_INFO_STREAM("LOAD " << mName << " limits from ROSPARAM: " << limits.max_velocity << " rad/s");
    }

    // Send joint limits information to board
    motor_t constraint;
    constraint.position = -1;
    constraint.velocity = (motor_control_t) limits.max_velocity*1000;
    constraint.current = -1;
    // Set type of command
    command.bitset.command = MOTOR_CONSTRAINT;

    // Build a packet
    message_abstract_u temp;
    temp.motor.motor = constraint;
    packet_information_t frame = CREATE_PACKET_DATA(command.command_message, HASHMAP_MOTOR, temp);
    // Add packet in the frame
    mSerial->addFrame(frame);

    joint_limits_interface::VelocityJointSoftLimitsHandle handle(joint_handle, // We read the state and read/write the command
                                                                 limits,       // Limits spec
                                                                 soft_limits);  // Soft limits spec

    vel_limits_interface.registerHandle(handle);
}

void Motor::run(diagnostic_updater::DiagnosticStatusWrapper &stat)
{

}


void Motor::motorFrame(unsigned char option, unsigned char type, unsigned char command, message_abstract_u message)
{
    ROS_INFO_STREAM("Motor decode " << mName );
    switch(command)
    {
        case MOTOR_MEASURE:

        break;
        case MOTOR_VEL_PID:
        ROS_INFO_STREAM("New PID parameter");
        break;
    }
}

}
