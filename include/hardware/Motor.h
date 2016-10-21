#ifndef MOTOR_H
#define MOTOR_H

#include <ros/ros.h>
#include <urdf/model.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <joint_limits_interface/joint_limits_interface.h>

#include <orbus_interface/MotorStatus.h>
#include <orbus_interface/ControlStatus.h>

#include <orbus_interface/UnavLimitsConfig.h>

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
    explicit Motor(const ros::NodeHandle &nh, orbus::serial_controller *serial, string name, unsigned int number);

    void initializeMotor();

    void run(diagnostic_updater::DiagnosticStatusWrapper &stat);

    void motorFrame(unsigned char option, unsigned char type, unsigned char command, motor_frame_u frame);

    void addRequestMeasure();

    void resetPosition(double position);

    void switchController(string type);

    void writeCommandsToHardware(ros::Duration period);

    void setupLimits(urdf::Model model);



    hardware_interface::JointStateHandle joint_state_handle;
    hardware_interface::JointHandle joint_handle;

private:
    static string convert_status(motor_state_t status);

    static motor_state_t get_state(string type);

    motor_t updateLimits(double position, double velocity, double effort);

    void reconfigureCB(orbus_interface::UnavLimitsConfig &config, uint32_t level);

private:
    //Initialization object
    //NameSpace for bridge controller
    ros::NodeHandle mNh;
    // Serial controller communication
    orbus::serial_controller *mSerial;
    // Name of the motor
    string mMotorName;
    unsigned int mNumber;
    // State of the motor
    motor_state_t mState, mDiagnosticState;
    double position;
    double velocity;
    double effort;
    double command;

    // Constraints
    motor_t constraints;
    // Reconfigure status
    orbus_interface::UnavLimitsConfig limits;

    /// ROS joint limits interface
    joint_limits_interface::VelocityJointSoftLimitsInterface vel_limits_interface;

    // Publisher diagnostic information
    ros::Publisher pub_status, pub_control, pub_measure, pub_reference;
    // Message
    orbus_interface::MotorStatus msg_status;
    orbus_interface::ControlStatus msg_reference, msg_measure, msg_control;

    // Number message
    motor_command_map_t motor_command;
    /// Load all configurators
    MotorPIDConfigurator *pid_velocity, *pid_current;
    MotorParamConfigurator *parameter;
    MotorEmergencyConfigurator *emergency;
    MotorDiagnosticConfigurator *diagnostic_current, *diagnostic_temperature;

    // Dynamic reconfigurator for limits
    dynamic_reconfigure::Server<orbus_interface::UnavLimitsConfig> *dsrv;
    boost::recursive_mutex config_mutex;
    /// Setup variable
    bool setup_;
    orbus_interface::UnavLimitsConfig last_config_, default_config_;
};

}

#endif // MOTOR_H
