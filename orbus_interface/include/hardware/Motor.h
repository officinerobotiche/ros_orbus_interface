#ifndef MOTOR_H
#define MOTOR_H

#include <ros/ros.h>
#include <urdf/model.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <joint_limits_interface/joint_limits_interface.h>

#include <orbus_msgs/MotorStatus.h>
#include <orbus_interface/ControlStatus.h>

#include "hardware/serial_controller.h"

#include "configurator/MotorPIDConfigurator.h"
#include "configurator/MotorParamConfigurator.h"
#include "configurator/MotorEmergencyConfigurator.h"
#include "configurator/MotorDiagnosticConfigurator.h"

using namespace std;

namespace ORInterface
{

class Motor : public diagnostic_updater::DiagnosticTask
{
public:
    explicit Motor(const ros::NodeHandle &nh, orbus::serial_controller *serial, unsigned int number);

    void initializeMotor();

    void run(diagnostic_updater::DiagnosticStatusWrapper &stat);

    void motorFrame(unsigned char option, unsigned char type, unsigned char command, motor_frame_u frame);

    void addRequestMeasure();

    void resetPosition(double position);

    void writeCommandsToHardware(ros::Duration period, double velocity_command);

    void setupLimits(hardware_interface::JointHandle joint_handle, boost::shared_ptr<urdf::ModelInterface> urdf);

    string mMotorName;

private:
    //Initialization object
    //NameSpace for bridge controller
    ros::NodeHandle mNh;
    // Serial controller communication
    orbus::serial_controller *mSerial;
    // Name of the motor
    string mName;
    unsigned int mNumber;

    /// ROS joint limits interface
    joint_limits_interface::VelocityJointSoftLimitsInterface vel_limits_interface;

    // Publisher diagnostic information
    ros::Publisher pub_status, pub_control, pub_measure, pub_reference;
    // Message
    orbus_msgs::MotorStatus msg_status;
    orbus_interface::ControlStatus msg_reference, msg_measure, msg_control;

    // Number message
    motor_command_map_t command;
    /// Load all configurators
    MotorPIDConfigurator *pid_velocity, *pid_current;
    MotorParamConfigurator *parameter;
    MotorEmergencyConfigurator *emergency;
    MotorDiagnosticConfigurator *diagnostic_current, *diagnostic_temperature;
};

}

#endif // MOTOR_H
