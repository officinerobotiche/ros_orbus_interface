
#include "hardware/Motor.h"

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>

#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

namespace ORInterface
{

Motor::Motor(const ros::NodeHandle& nh, orbus::serial_controller *serial, string name, unsigned int number)
    : DiagnosticTask(name + "_status")
    , joint_state_handle(name, &position, &velocity, &effort)
    , joint_handle(joint_state_handle, &command)
    , mNh(nh)
    , mSerial(serial)
{
    motor_command.bitset.motor = number;
    mNumber = number;

    mMotorName = name;

    pid_velocity = new MotorPIDConfigurator(nh, serial, mMotorName, "velocity", MOTOR_VEL_PID, number);
    pid_current = new MotorPIDConfigurator(nh, serial, mMotorName, "current", MOTOR_CURRENT_PID, number);
    parameter = new MotorParamConfigurator(nh, serial, mMotorName, number);
    emergency = new MotorEmergencyConfigurator(nh, serial, mMotorName, number);
    diagnostic_current = new MotorDiagnosticConfigurator(nh, serial, mMotorName, "current", MOTOR_SAFETY, number);
    diagnostic_temperature = new MotorDiagnosticConfigurator(nh, serial, mMotorName, "temperature", 0xFFFF, number);

    // Add a status motor publisher
    pub_status = mNh.advertise<orbus_interface::MotorStatus>(mMotorName + "/status", 10);

    pub_reference = mNh.advertise<orbus_interface::ControlStatus>(mMotorName + "/reference", 10,
            boost::bind(&Motor::connectionCallback, this, _1), boost::bind(&Motor::connectionCallback, this, _1));
    pub_measure = mNh.advertise<orbus_interface::ControlStatus>(mMotorName + "/measure", 10);
    pub_control = mNh.advertise<orbus_interface::ControlStatus>(mMotorName + "/control", 10,
            boost::bind(&Motor::connectionCallback, this, _1), boost::bind(&Motor::connectionCallback, this, _1));

    //Load limits dynamic reconfigure
    dsrv = new dynamic_reconfigure::Server<orbus_interface::UnavLimitsConfig>(config_mutex, ros::NodeHandle("~" + mMotorName + "/limits"));
    dynamic_reconfigure::Server<orbus_interface::UnavLimitsConfig>::CallbackType cb = boost::bind(&Motor::reconfigureCB, this, _1, _2);
    dsrv->setCallback(cb);
}

void Motor::connectionCallback(const ros::SingleSubscriberPublisher& pub)
{
    ROS_DEBUG_STREAM("Update: " << pub.getSubscriberName() << " - " << pub.getTopic());
    // Clear list to send
    information_motor.clear();
    ROS_DEBUG_STREAM("Num referecence: " << pub_reference.getNumSubscribers());
    if(pub_reference.getNumSubscribers() >= 1)
    {
        // Motor control
        motor_command.bitset.command = MOTOR_REFERENCE;
        // Build a packet
        packet_information_t frame_reference = CREATE_PACKET_RESPONSE(motor_command.command_message, HASHMAP_MOTOR, PACKET_REQUEST);
        information_motor.push_back(frame_reference);
    }
    ROS_DEBUG_STREAM("Num control: " << pub_control.getNumSubscribers());
    if(pub_control.getNumSubscribers() >= 1)
    {
        // Motor control
        motor_command.bitset.command = MOTOR_CONTROL;
        // Build a packet
        packet_information_t frame_control = CREATE_PACKET_RESPONSE(motor_command.command_message, HASHMAP_MOTOR, PACKET_REQUEST);
        information_motor.push_back(frame_control);
    }
}

void Motor::initializeMotor()
{
    // Initialize all parameters
    pid_velocity->initConfigurator();
    pid_current->initConfigurator();
    parameter->initConfigurator();
    emergency->initConfigurator();
    // Initialize ONLY diagnostic current
    diagnostic_current->initConfigurator();

    // Set type of command
    motor_command.bitset.command = MOTOR_CONSTRAINT;
    // Build a packet
    message_abstract_u temp;
    temp.motor.motor = constraints;
    packet_information_t frame_constraints = CREATE_PACKET_DATA(motor_command.command_message, HASHMAP_MOTOR, temp);
    // Add packet in the frame
    mSerial->addFrame(frame_constraints);
}

motor_t Motor::updateLimits(double position, double velocity, double effort)
{
    // Constraints
    motor_t constraints;

    constraints.position = position;
    constraints.velocity = (motor_control_t) (velocity * 1000.0);
    constraints.current = (motor_control_t) (last_config_.current * 1000.0);
    constraints.effort = (motor_control_t) (effort * 1000.0);
    constraints.pwm = (motor_control_t) last_config_.PWM;

    last_config_.position = position;
    last_config_.velocity = velocity;
    last_config_.effort = effort;

    dsrv->updateConfig(last_config_);

    ROS_DEBUG_STREAM("LIMITS param [pos:" << constraints.position << ", vel:" << constraints.velocity << ", curr:" << constraints.current << ", eff:" << constraints.effort <<", PWM:" << constraints.pwm << "]");

    return constraints;
}

void Motor::reconfigureCB(orbus_interface::UnavLimitsConfig &config, uint32_t level)
{
    //The first time we're called, we just want to make sure we have the
    //original configuration
    if(!setup_)
    {
      last_config_ = config;
      default_config_ = last_config_;
      setup_ = true;
      return;
    }

    if(config.restore_defaults) {
      config = default_config_;
      //if someone sets restore defaults on the parameter server, prevent looping
      config.restore_defaults = false;
    }

    // Update last config
    last_config_ = config;

    // Update limits
    constraints = updateLimits(config.position, config.velocity, config.effort);

    // Set type of command
    motor_command.bitset.command = MOTOR_CONSTRAINT;
    // Build a packet
    message_abstract_u temp;
    temp.motor.motor = constraints;
    packet_information_t frame = CREATE_PACKET_DATA(motor_command.command_message, HASHMAP_MOTOR, temp);
    // Add packet in the frame
    mSerial->addFrame(frame);
}

void Motor::setupLimits(urdf::Model model)
{
    /// Add a velocity joint limits infomations
    joint_limits_interface::JointLimits limits;
    joint_limits_interface::SoftJointLimits soft_limits;

    bool state = true;

    // Manual value setting
    limits.has_velocity_limits = true;
    limits.max_velocity = 5.0;
    limits.has_effort_limits = true;
    limits.max_effort = 2.0;

    // Populate (soft) joint limits from URDF
    // Limits specified in URDF overwrite existing values in 'limits' and 'soft_limits'
    // Limits not specified in URDF preserve their existing values

    boost::shared_ptr<const urdf::Joint> urdf_joint = model.getJoint(mMotorName);
    const bool urdf_limits_ok = getJointLimits(urdf_joint, limits);
    const bool urdf_soft_limits_ok = getSoftJointLimits(urdf_joint, soft_limits);

    if(urdf_limits_ok) {
        ROS_INFO_STREAM("LOAD [" << mMotorName << "] limits from URDF: |" << limits.max_velocity << "| rad/s & |" << limits.max_effort << "| Nm");
        state = false;
    }

    if(urdf_soft_limits_ok) {
        ROS_INFO_STREAM("LOAD [" << mMotorName << "] soft limits from URDF: |" << limits.max_velocity << "| rad/s & |" << limits.max_effort << "| Nm");
        state = false;
    }

    // Populate (soft) joint limits from the ros parameter server
    // Limits specified in the parameter server overwrite existing values in 'limits' and 'soft_limits'
    // Limits not specified in the parameter server preserve their existing values
    const bool rosparam_limits_ok = getJointLimits(mMotorName, mNh, limits);
    if(rosparam_limits_ok) {
        ROS_WARN_STREAM("OVERLOAD [" << mMotorName << "] limits from ROSPARAM: |" << limits.max_velocity << "| rad/s & |" << limits.max_effort << "| Nm");
        state = false;
    }
    else
    {
        ROS_DEBUG("Setup limits, PARAM NOT available");
    }
    // If does not read any parameter from URDF or rosparm load default parameter
    if(state)
    {
        ROS_WARN_STREAM("LOAD [" << mMotorName << "] with DEFAULT limit = |" << limits.max_velocity << "| rad/s & |" << limits.max_effort << "| Nm");
    }

    // Set maximum limits if doesn't have limit
    if(limits.has_position_limits == false)
    {
        limits.max_position = 6.28;
    }
    // Update limits
    constraints = updateLimits(limits.max_position, limits.max_velocity, limits.max_effort);

    // Set type of command
    motor_command.bitset.command = MOTOR_CONSTRAINT;
    // Build a packet
    message_abstract_u temp;
    temp.motor.motor = constraints;
    packet_information_t frame = CREATE_PACKET_DATA(motor_command.command_message, HASHMAP_MOTOR, temp);
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
    motor_command.bitset.command = MOTOR_DIAGNOSTIC;
    // Build a packet
    packet_information_t frame = CREATE_PACKET_RESPONSE(motor_command.command_message, HASHMAP_MOTOR, PACKET_REQUEST);
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
    stat.add("Current (A)", fabs(msg_measure.current));
    stat.add("Torque (Nm)", msg_measure.effort);

    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Motor Ready!");

    if(mDiagnosticState < STATE_CONTROL_DISABLE)
    {
        stat.mergeSummaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "%s control", msg_status.state.c_str());
    }
    else
    {
        stat.mergeSummaryf(diagnostic_msgs::DiagnosticStatus::OK, "%s", msg_status.state.c_str());
    }

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

    if (fabs(msg_measure.current) > diagnostic_current->levels.critical)
    {
        stat.mergeSummaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Critical current: %5.2f > %5.2f A", msg_measure.current, diagnostic_current->levels.critical);
    }
    else if (fabs(msg_measure.current) > diagnostic_current->levels.warning)
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
    case STATE_CONTROL_SAFETY:
        return "Safety";
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
    ROS_DEBUG_STREAM("Motor decode " << mMotorName );
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
        // Update joint status
        effort = msg_measure.effort;
        position += frame.motor.position_delta;
        velocity = msg_measure.velocity;
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
        mDiagnosticState = frame.diagnostic.state;
        msg_status.state = convert_status(frame.diagnostic.state);
        msg_status.watt = (frame.diagnostic.watt/1000.0); /// in W
        msg_status.time_execution = frame.diagnostic.time_control;
        msg_status.voltage = (frame.diagnostic.volt/1000.0); /// in V;
        msg_status.temperature = frame.diagnostic.temperature;
        // publish a message
        msg_status.header.stamp = ros::Time::now();
        pub_status.publish(msg_status);
        break;
    case MOTOR_STATE:
        if(option == PACKET_DATA)
        {
            mState = frame.state;
            //ROS_INFO_STREAM("Motor state: " << convert_status(mState));
        } else if(option != PACKET_ACK) {
            ROS_ERROR_STREAM("ERROR "<< option << " Motor[" << mNumber << "] message \""<< command << "\"=(" << (int) command << ")");
        }
        break;
    case MOTOR_VEL_PID:
        if(option == PACKET_DATA)
        {
            ROS_INFO_STREAM("Velocity PID frame");
            pid_velocity->setParam(frame.pid);
        } else if(option != PACKET_ACK) {
            ROS_ERROR_STREAM("ERROR "<< option << " Motor[" << mNumber << "] message \""<< command << "\"=(" << (int) command << ")");
        }
        break;
    case MOTOR_CURRENT_PID:
        if(option == PACKET_DATA)
        {
            ROS_INFO_STREAM("Current PID frame");
            pid_current->setParam(frame.pid);
        } else if(option != PACKET_ACK) {
            ROS_ERROR_STREAM("ERROR "<< option << " Motor[" << mNumber << "] message \""<< command << "\"=(" << (int) command << ")");
        }
        break;
    case MOTOR_EMERGENCY:
        if(option == PACKET_DATA)
        {
            ROS_INFO_STREAM("Emergency frame");
            emergency->setParam(frame.emergency);
        } else if(option != PACKET_ACK) {
            ROS_ERROR_STREAM("ERROR "<< option << " Motor[" << mNumber << "] message \""<< command << "\"=(" << (int) command << ")");
        }
        break;
    case MOTOR_PARAMETER:
        if(option == PACKET_DATA)
        {
            ROS_INFO_STREAM("Parameter frame");
            parameter->setParam(frame.parameter);
        } else if(option != PACKET_ACK) {
            ROS_ERROR_STREAM("ERROR "<< option << " Motor[" << mNumber << "] message \""<< command << "\"=(" << (int) command << ")");
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
    motor_command.bitset.command = MOTOR_MEASURE;
    // Build a packet
    packet_information_t frame_measure = CREATE_PACKET_RESPONSE(motor_command.command_message, HASHMAP_MOTOR, PACKET_REQUEST);
    // Add packet in the frame
    mSerial->addFrame(frame_measure)->addFrame(information_motor)->sendList();
}

void Motor::resetPosition(double position)
{
    // Set type of command
    motor_command.bitset.command = MOTOR_POS_RESET;
    // Build a packet
    message_abstract_u temp;
    temp.motor.reference = static_cast<motor_control_t>(position*1000.0);
    packet_information_t frame = CREATE_PACKET_DATA(motor_command.command_message, HASHMAP_MOTOR, temp);
    // Add packet in the frame
    mSerial->addFrame(frame);
}

motor_state_t Motor::get_state(string type)
{
    motor_state_t state;
    if(type.compare("diff_drive_controller/DiffDriveController") == 0)
    {
        state = STATE_CONTROL_VELOCITY;
    }
    else
    {
        state = STATE_CONTROL_DISABLE;
    }
    ROS_DEBUG_STREAM("type:" << type << " - state: " << (int) state);
    return state;
}

void Motor::switchController(string type)
{
    // Update the new state
    mState = get_state(type);

    // Set type of command
    motor_command.bitset.command = MOTOR_STATE;
    // Build a packet
    message_abstract_u temp;
    temp.motor.state = mState;
    packet_information_t frame = CREATE_PACKET_DATA(motor_command.command_message, HASHMAP_MOTOR, temp);
    // Add packet in the frame
    mSerial->addFrame(frame);
}

void Motor::writeCommandsToHardware(ros::Duration period)
{
    switch(mState)
    {
    case STATE_CONTROL_VELOCITY:
        // Enforce joint limits for all registered handles
        // Note: one can also enforce limits on a per-handle basis: handle.enforceLimits(period)
        vel_limits_interface.enforceLimits(period);
        // Set type of command
        motor_command.bitset.command = MOTOR_VEL_REF;
        break;
    case STATE_CONTROL_CURRENT:
        // Set type of command
        motor_command.bitset.command = MOTOR_CURRENT_REF;
        break;
    case STATE_CONTROL_DISABLE:
        // Does not send any command
        return;
    }

    long long int reference_long = static_cast<long long int>(command*1000.0);
    motor_control_t reference;
    // >>>>> Saturation on 32 bit values
    if(reference_long > MOTOR_CONTROL_MAX) {
        reference = MOTOR_CONTROL_MAX;
    } else if (reference_long < MOTOR_CONTROL_MIN) {
        reference_long = MOTOR_CONTROL_MIN;
    } else {
        reference = (motor_control_t) reference_long;
    }
    // <<<<< Saturation on 32 bit values

    //ROS_INFO_STREAM("Vel[" << mNumber << "]:" << velocity);
    // Build a packet
    message_abstract_u temp;
    temp.motor.reference = reference;
    packet_information_t frame = CREATE_PACKET_DATA(motor_command.command_message, HASHMAP_MOTOR, temp);
    // Add packet in the frame
    mSerial->addFrame(frame);

}

}
