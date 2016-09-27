
#include "hardware/Motor.h"

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>

#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

namespace ORInterface
{

Motor::Motor(const ros::NodeHandle& nh, orbus::serial_controller *serial, unsigned int number)
    : DiagnosticTask("motor_" + to_string(number) + "_status")
    , mNh(nh)
    , mSerial(serial)
    , mName("motor_" + to_string(number))
{
    command.bitset.motor = number;
    mNumber = number;

    pid_velocity = new MotorPIDConfigurator(nh, serial, mName, "velocity", MOTOR_VEL_PID, number);
    pid_current = new MotorPIDConfigurator(nh, serial, mName, "current", MOTOR_CURRENT_PID, number);
    parameter = new MotorParamConfigurator(nh, serial, mName, number);
    emergency = new MotorEmergencyConfigurator(nh, serial, mName, number);
    diagnostic_current = new MotorDiagnosticConfigurator(nh, serial, mName, "current", number);
    diagnostic_temperature = new MotorDiagnosticConfigurator(nh, serial, mName, "temperature", number);

}

void Motor::initializeMotor()
{
    pid_velocity->initConfigurator();
    pid_current->initConfigurator();
    parameter->initConfigurator();
    emergency->initConfigurator();
}

void Motor::registerControlInterfaces(hardware_interface::JointStateInterface joint_state_interface, hardware_interface::VelocityJointInterface velocity_joint_interface, boost::shared_ptr<urdf::ModelInterface> urdf)
{
    /// Joint hardware interface
    hardware_interface::JointStateHandle joint_state_handle(mName, &position, &velocity, &effort);

    joint_state_interface.registerHandle(joint_state_handle);

    /// Differential drive interface
    hardware_interface::JointHandle joint_handle(joint_state_handle, &velocity_command);
    velocity_joint_interface.registerHandle(joint_handle);

    setupLimits(joint_handle, urdf);
}

void Motor::setupLimits(hardware_interface::JointHandle joint_handle, boost::shared_ptr<urdf::ModelInterface> urdf)
{
    /// Add a velocity joint limits infomations
    /// Populate with any of the methods presented in the previous example...
    joint_limits_interface::JointLimits limits;
    joint_limits_interface::SoftJointLimits soft_limits;

    // Manual value setting
    limits.has_velocity_limits = true;
    limits.max_velocity = 5.0;
    bool state = true;

    // Populate (soft) joint limits from URDF
    // Limits specified in URDF overwrite existing values in 'limits' and 'soft_limits'
    // Limits not specified in URDF preserve their existing values
    if(urdf != NULL) {
        boost::shared_ptr<const urdf::Joint> urdf_joint = urdf->getJoint(mName);
        const bool urdf_limits_ok = getJointLimits(urdf_joint, limits);
        const bool urdf_soft_limits_ok = getSoftJointLimits(urdf_joint, soft_limits);
        if(urdf_limits_ok) {
            ROS_INFO_STREAM("LOAD " << mName << " limits from URDF: |" << limits.max_velocity << "| rad/s");
        }
        if(urdf_soft_limits_ok) {
            ROS_INFO_STREAM("LOAD " << mName << " soft limits from URDF: |" << limits.max_velocity << "| rad/s");
        }
        state = false;
    }
    else
    {
        ROS_WARN("Setup limits, URDF NOT available");
    }

    // Populate (soft) joint limits from the ros parameter server
    // Limits specified in the parameter server overwrite existing values in 'limits' and 'soft_limits'
    // Limits not specified in the parameter server preserve their existing values
    const bool rosparam_limits_ok = getJointLimits(mName, mNh, limits);
    if(rosparam_limits_ok) {
        ROS_INFO_STREAM("LOAD " << mName << " limits from ROSPARAM: |" << limits.max_velocity << "| rad/s");
        state = false;
    }
    else
    {
        ROS_WARN("Setup limits, PARAM NOT available");
    }
    // If does not read any parameter from URDF or rosparm load default parameter
    if(state)
    {
        ROS_INFO_STREAM("Load " << mName << " with limit = |" << limits.max_velocity << "| rad/s");
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
    // Set type of command
    command.bitset.command = MOTOR_DIAGNOSTIC;
    // Build a packet
    packet_information_t frame = CREATE_PACKET_RESPONSE(command.command_message, HASHMAP_MOTOR, PACKET_REQUEST);
    // Add packet in the frame
    if(mSerial->addFrame(frame)->sendList())
    {
        ROS_DEBUG_STREAM("Request Diagnostic COMPLETED from:" << mName << " in uNav");
    }
    else
    {
        ROS_ERROR_STREAM("Unable to receive packet from uNav");
    }

    stat.add("State ", (int) status_msg.state);
    stat.add("PWM rate (%)", status_msg.pwm);
    stat.add("Position (deg)", ((double)status_msg.position) * 180.0/M_PI);
    stat.add("Velociy (rad/s)", status_msg.velocity);
    stat.add("Torque (Nm)", status_msg.effort);

    stat.add("Current (A)", status_msg.current);
    stat.add("Voltage (V)", status_msg.voltage);
    stat.add("Watt (W)", status_msg.watt);
    stat.add("Temperature (°C)", status_msg.temperature);

    stat.add("Time execution (nS)", status_msg.time_execution);

    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Motor Ready!");

    if (status_msg.temperature > diagnostic_temperature->levels.critical)
    {
        stat.mergeSummaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Critical temperature: %5.2f > %5.2f °C", status_msg.temperature, diagnostic_temperature->levels.critical);
    }
    else if (status_msg.temperature > diagnostic_temperature->levels.warning)
    {
        stat.mergeSummaryf(diagnostic_msgs::DiagnosticStatus::WARN, "Temperature over: %5.2f > %5.2f °C", status_msg.temperature, diagnostic_temperature->levels.warning);
    }
    else
    {
        stat.mergeSummaryf(diagnostic_msgs::DiagnosticStatus::OK, "Temperature OK: %5.2f °C", status_msg.temperature);
    }

    if (status_msg.current > diagnostic_current->levels.critical)
    {
        stat.mergeSummaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Critical current: %5.2f > %5.2f A", status_msg.current, diagnostic_current->levels.critical);
    }
    else if (status_msg.current > diagnostic_current->levels.warning)
    {
        stat.mergeSummaryf(diagnostic_msgs::DiagnosticStatus::WARN, "Current over %5.2f > %5.2f A", status_msg.current, diagnostic_current->levels.warning);
    }
    else
    {
        stat.mergeSummaryf(diagnostic_msgs::DiagnosticStatus::OK, "Current OK: %5.2f A", status_msg.current);
    }
}


void Motor::motorFrame(unsigned char option, unsigned char type, unsigned char command, motor_frame_u frame)
{
    ROS_DEBUG_STREAM("Motor decode " << mName );
    switch(command)
    {
        case MOTOR_MEASURE:
        status_msg.current = frame.motor.current;
        status_msg.effort = frame.motor.current; ///< TODO Change with a good estimation
        status_msg.position = frame.motor.position;
        position += frame.motor.position_delta;
        status_msg.pwm = ((double) frame.motor.pwm) * 100.0 / INT16_MAX;
        status_msg.state = frame.motor.state;
        velocity = ((double)frame.motor.velocity) / 1000.0;
        status_msg.velocity = velocity;
        break;
        case MOTOR_DIAGNOSTIC:
        status_msg.watt = (frame.diagnostic.watt/1000.0); /// in W
        status_msg.time_execution = frame.diagnostic.time_control;
        status_msg.voltage = (frame.diagnostic.volt/1000.0); /// in V;
        status_msg.temperature = frame.diagnostic.temperature;
        break;
        case MOTOR_VEL_PID:
        if(option == PACKET_DATA)
        {
            ROS_INFO_STREAM("Velocity PID frame");
            pid_velocity->setParam(frame.pid);
        }
        break;
        case MOTOR_CURRENT_PID:
        if(option == PACKET_DATA)
        {
            ROS_INFO_STREAM("Current PID frame");
            pid_current->setParam(frame.pid);
        }
        break;
        case MOTOR_EMERGENCY:
        if(option == PACKET_DATA)
        {
            ROS_INFO_STREAM("Emergency frame");
            emergency->setParam(frame.emergency);
        }
        break;
        case MOTOR_PARAMETER:
        if(option == PACKET_DATA)
        {
            ROS_INFO_STREAM("Parameter frame");
            parameter->setParam(frame.parameter);
        }
        break;
    }
}

void Motor::addRequestMeasure()
{
    // Set type of command
    command.bitset.command = MOTOR_MEASURE;
    // Build a packet
    packet_information_t frame = CREATE_PACKET_RESPONSE(command.command_message, HASHMAP_MOTOR, PACKET_REQUEST);
    // Add packet in the frame
    mSerial->addFrame(frame);
}

void Motor::writeCommandsToHardware(ros::Duration period)
{
    // Enforce joint limits for all registered handles
    // Note: one can also enforce limits on a per-handle basis: handle.enforceLimits(period)
    vel_limits_interface.enforceLimits(period);

    long int velocity_long = static_cast<long int>(velocity_command*1000.0);
    motor_control_t velocity;
    // >>>>> Saturation on 16 bit values
    if(velocity_long > MOTOR_CONTROL_MAX) {
        velocity = MOTOR_CONTROL_MAX;
    } else if (velocity_long < MOTOR_CONTROL_MIN) {
        velocity_long = MOTOR_CONTROL_MIN;
    } else {
        velocity = (motor_control_t) velocity_long;
    }
    // <<<<< Saturation on 16 bit values

    // Set type of command
    command.bitset.command = MOTOR_VEL_REF;
    // Build a packet
    message_abstract_u temp;
    temp.motor.reference = velocity;
    packet_information_t frame = CREATE_PACKET_DATA(command.command_message, HASHMAP_MOTOR, temp);
    // Add packet in the frame
    mSerial->addFrame(frame);

}

}
