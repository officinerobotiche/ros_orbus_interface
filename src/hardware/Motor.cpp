
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

    //Initialize the name of the motor
    if(nh.hasParam(mName + "/name_joint"))
    {
        nh.getParam(mName + "/name_joint", mMotorName);
    }
    else
    {
        nh.setParam(mName + "/name_joint", mName);
        mMotorName = mName;
    }

    pid_velocity = new MotorPIDConfigurator(nh, serial, mName, "velocity", MOTOR_VEL_PID, number);
    pid_current = new MotorPIDConfigurator(nh, serial, mName, "current", MOTOR_CURRENT_PID, number);
    parameter = new MotorParamConfigurator(nh, serial, mName, number);
    emergency = new MotorEmergencyConfigurator(nh, serial, mName, number);
    diagnostic_current = new MotorDiagnosticConfigurator(nh, serial, mName, "current", number);
    diagnostic_temperature = new MotorDiagnosticConfigurator(nh, serial, mName, "temperature", number);

    // Add a status motor publisher
    pub_status = mNh.advertise<orbus_interface::MotorStatus>(mName + "/status", 10);

    pub_reference = mNh.advertise<orbus_interface::ControlStatus>(mName + "/reference", 10);
    pub_measure = mNh.advertise<orbus_interface::ControlStatus>(mName + "/measure", 10);
    pub_control = mNh.advertise<orbus_interface::ControlStatus>(mName + "/control", 10);

}

void Motor::initializeMotor()
{
    pid_velocity->initConfigurator();
    pid_current->initConfigurator();
    parameter->initConfigurator();
    emergency->initConfigurator();
}

void Motor::setupLimits(hardware_interface::JointHandle joint_handle, boost::shared_ptr<urdf::ModelInterface> urdf)
{
    /// Add a velocity joint limits infomations
    joint_limits_interface::JointLimits limits;
    joint_limits_interface::SoftJointLimits soft_limits;

    // Manual value setting
    limits.has_velocity_limits = true;
    limits.max_velocity = 5.0;
    bool state = true;
//    limits.has_effort_limits = true;
//    limits.max_effort = 2.0;

    // Populate (soft) joint limits from URDF
    // Limits specified in URDF overwrite existing values in 'limits' and 'soft_limits'
    // Limits not specified in URDF preserve their existing values
    if(urdf != NULL) {
        boost::shared_ptr<const urdf::Joint> urdf_joint = urdf->getJoint(mMotorName);
        const bool urdf_limits_ok = getJointLimits(urdf_joint, limits);
        const bool urdf_soft_limits_ok = getSoftJointLimits(urdf_joint, soft_limits);
        if(urdf_limits_ok) {
            ROS_INFO_STREAM("LOAD [" << mMotorName << "] limits from URDF: |" << limits.max_velocity << "| rad/s");
        }
        if(urdf_soft_limits_ok) {
            ROS_INFO_STREAM("LOAD [" << mMotorName << "] soft limits from URDF: |" << limits.max_velocity << "| rad/s");
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
    const bool rosparam_limits_ok = getJointLimits(mMotorName, mNh, limits);
    if(rosparam_limits_ok) {
        ROS_WARN_STREAM("OVERLOAD [" << mMotorName << "] limits from ROSPARAM: |" << limits.max_velocity << "| rad/s");
        state = false;
    }
    else
    {
        ROS_DEBUG("Setup limits, PARAM NOT available");
    }
    // If does not read any parameter from URDF or rosparm load default parameter
    if(state)
    {
        ROS_INFO_STREAM("Load [" << mMotorName << "] with limit = |" << limits.max_velocity << "| rad/s");
    }

    // Send joint limits information to board
    motor_t constraint;
    constraint.position = MOTOR_CONTROL_MAX;
    constraint.velocity = (motor_control_t) limits.max_velocity*1000;
    constraint.current = MOTOR_CONTROL_MAX;

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
        ROS_DEBUG_STREAM("Request Diagnostic COMPLETED from:" << mMotorName << " in uNav");
    }
    else
    {
        ROS_ERROR_STREAM("Unable to receive packet from uNav");
    }

    stat.add("State ", msg_status.state);
    stat.add("PWM rate (%)", msg_measure.pwm);
    stat.add("Voltage (V)", msg_status.voltage);
    stat.add("Watt (W)", msg_status.watt);
    stat.add("Temperature (°C)", msg_status.temperature);
    stat.add("Time execution (nS)", msg_status.time_execution);

    stat.add("Position (deg)", ((double)msg_measure.position) * 180.0/M_PI);
    stat.add("Velociy (RPM)", ((double)msg_measure.velocity) * (30.0 / M_PI));
    stat.add("Current (A)", msg_measure.current);
    stat.add("Torque (Nm)", msg_measure.effort);

    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Motor Ready!");

    if (msg_status.temperature > diagnostic_temperature->levels.critical)
    {
        stat.mergeSummaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Critical temperature: %5.2f > %5.2f °C", msg_status.temperature, diagnostic_temperature->levels.critical);
    }
    else if (msg_status.temperature > diagnostic_temperature->levels.warning)
    {
        stat.mergeSummaryf(diagnostic_msgs::DiagnosticStatus::WARN, "Temperature over: %5.2f > %5.2f °C", msg_status.temperature, diagnostic_temperature->levels.warning);
    }
    else
    {
        stat.mergeSummaryf(diagnostic_msgs::DiagnosticStatus::OK, "Temperature OK: %5.2f °C", msg_status.temperature);
    }

    if (msg_measure.current > diagnostic_current->levels.critical)
    {
        stat.mergeSummaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Critical current: %5.2f > %5.2f A", msg_measure.current, diagnostic_current->levels.critical);
    }
    else if (msg_measure.current > diagnostic_current->levels.warning)
    {
        stat.mergeSummaryf(diagnostic_msgs::DiagnosticStatus::WARN, "Current over %5.2f > %5.2f A", msg_measure.current, diagnostic_current->levels.warning);
    }
    else
    {
        stat.mergeSummaryf(diagnostic_msgs::DiagnosticStatus::OK, "Current OK: %5.2f A", msg_measure.current);
    }
}

string Motor::convert_status(motor_state_t status)
{
    switch(status)
    {
    case STATE_CONTROL_EMERGENCY:
        return "Emergency";
    case STATE_CONTROL_DISABLE:
        return "Disabled";
    case STATE_CONTROL_POSITION:
        return "Position control";
    case STATE_CONTROL_VELOCITY:
        return "Velocity control";
    case STATE_CONTROL_CURRENT:
        return "Current control";
    case STATE_CONTROL_DIRECT:
        return "Direct";
    default:
        return "Unknown";
    }
}

void Motor::motorFrame(unsigned char option, unsigned char type, unsigned char command, motor_frame_u frame)
{
    ROS_DEBUG_STREAM("Motor decode " << mName );
    switch(command)
    {
    case MOTOR_MEASURE:
       // ROS_INFO_STREAM("Measure Motor[" << mNumber << "] current: " << frame.motor.current);
        msg_measure.pwm = ((double) frame.motor.pwm) * 100.0 / 2048;
        msg_measure.position = frame.motor.position;
        msg_measure.velocity = ((double)frame.motor.velocity) / 1000.0;
        msg_measure.current = ((double) frame.motor.current) / 1000.0;
        msg_measure.effort = ((double) frame.motor.effort) / 1000.0;
        // publish a message
        msg_measure.header.stamp = ros::Time::now();
        pub_measure.publish(msg_measure);
        break;
    case MOTOR_CONTROL:
        // ROS_INFO_STREAM("Control Motor[" << mNumber << "] current: " << frame.motor.current);
        msg_control.position = frame.motor.position;
        msg_control.velocity = ((double)frame.motor.velocity) / 1000.0;
        msg_control.current = ((double) frame.motor.current) / 1000.0;
        // publish a message
        msg_control.header.stamp = ros::Time::now();
        pub_control.publish(msg_control);
        break;
    case MOTOR_REFERENCE:
        // ROS_INFO_STREAM("Reference Motor[" << mNumber << "] current: " << frame.motor.current);
        msg_reference.pwm = ((double) frame.motor.pwm) * 100.0 / 2048;
        msg_reference.position = frame.motor.position;
        msg_reference.velocity = ((double)frame.motor.velocity) / 1000.0;
        msg_reference.current = ((double) frame.motor.current) / 1000.0;
        // publish a message
        msg_reference.header.stamp = ros::Time::now();
        pub_reference.publish(msg_reference);
        break;
    case MOTOR_DIAGNOSTIC:
        msg_status.state = convert_status(frame.diagnostic.state);
        msg_status.watt = (frame.diagnostic.watt/1000.0); /// in W
        msg_status.time_execution = frame.diagnostic.time_control;
        msg_status.voltage = (frame.diagnostic.volt/1000.0); /// in V;
        msg_status.temperature = frame.diagnostic.temperature;
        // publish a message
        msg_status.header.stamp = ros::Time::now();
        pub_status.publish(msg_status);
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
    default:
        if(option != PACKET_ACK)
        {
            ROS_ERROR_STREAM("Motor[" << mNumber << "] message \""<< command << "\"=(" << (int) command << ")" << " does not implemented!");
        }
        break;
    }
}

void Motor::addRequestMeasure()
{
    // Set type of command
    command.bitset.command = MOTOR_MEASURE;
    // Build a packet
    packet_information_t frame_measure = CREATE_PACKET_RESPONSE(command.command_message, HASHMAP_MOTOR, PACKET_REQUEST);
    // Motor control
    command.bitset.command = MOTOR_CONTROL;
    // Build a packet
    packet_information_t frame_control = CREATE_PACKET_RESPONSE(command.command_message, HASHMAP_MOTOR, PACKET_REQUEST);
    // Motor control
    command.bitset.command = MOTOR_REFERENCE;
    // Build a packet
    packet_information_t frame_reference = CREATE_PACKET_RESPONSE(command.command_message, HASHMAP_MOTOR, PACKET_REQUEST);
    // Add packet in the frame
    mSerial->addFrame(frame_measure)->addFrame(frame_control)->addFrame(frame_reference)->sendList();
}

void Motor::resetPosition(double position)
{
    // Set type of command
    command.bitset.command = MOTOR_POS_RESET;
    // Build a packet
    message_abstract_u temp;
    temp.motor.reference = static_cast<motor_control_t>(position*1000.0);
    packet_information_t frame = CREATE_PACKET_DATA(command.command_message, HASHMAP_MOTOR, temp);
    // Add packet in the frame
    mSerial->addFrame(frame);
}

void Motor::writeCommandsToHardware(ros::Duration period, double velocity_command)
{
    // Enforce joint limits for all registered handles
    // Note: one can also enforce limits on a per-handle basis: handle.enforceLimits(period)
    vel_limits_interface.enforceLimits(period);

    long long int velocity_long = static_cast<long long int>(velocity_command*1000.0);
    motor_control_t velocity;
    // >>>>> Saturation on 32 bit values
    if(velocity_long > MOTOR_CONTROL_MAX) {
        velocity = MOTOR_CONTROL_MAX;
    } else if (velocity_long < MOTOR_CONTROL_MIN) {
        velocity_long = MOTOR_CONTROL_MIN;
    } else {
        velocity = (motor_control_t) velocity_long;
    }
    // <<<<< Saturation on 32 bit values
    //ROS_INFO_STREAM("Vel[" << mNumber << "]:" << velocity);
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
