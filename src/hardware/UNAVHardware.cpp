/* 
 * File:   ROSMotionController.cpp
 * Author: Raffaello Bonghi
 * 
 * Created on 15 November 2013, 18:34
 */

#include "hardware/UNAVHardware.h"
#include <limits>

#include <boost/assign/list_of.hpp>

#define NUMBER_PUB 10
#define SGN(x)  ( ((x) < 0) ?  -1 : ( ((x) == 0 ) ? 0 : 1) )

using namespace std;

namespace
{
  const uint8_t LEFT = 0, RIGHT = 1;
}

UNAVHardware::UNAVHardware(const ros::NodeHandle& nh, ParserPacket* serial)
: ORBHardware(nh, serial) {

    string param_type_board = "Motor Control";
    if (type_board.compare(param_type_board) == 0) {
        nh_.setParam("info/type_board", type_board);
    } else {
        throw (controller_exception("Other board: " + type_board));
    }

    serial->addCallback(&UNAVHardware::motionPacket, this, HASHMAP_MOTION);
    serial->addCallback(&UNAVHardware::motorPacket, this, HASHMAP_MOTOR);
    addVectorPacketRequest(&UNAVHardware::updatePacket, this);
    addParameterPacketRequest(&UNAVHardware::addParameter, this);
    addTimerEvent(&UNAVHardware::timerEvent, this);

    //Open Service
    srv_pid = nh_.advertiseService("pid", &UNAVHardware::pidServiceCallback, this);
    srv_parameter = nh_.advertiseService("parameter", &UNAVHardware::parameterServiceCallback, this);
    srv_constraint = nh_.advertiseService("constraint", &UNAVHardware::constraintServiceCallback, this);
    srv_emergency = nh_.advertiseService("emergency", &UNAVHardware::emergencyServiceCallback, this);

    registerControlInterfaces();

    joints_[LEFT].state = STATE_CONTROL_DISABLE;
    joints_[RIGHT].state = STATE_CONTROL_DISABLE;
}

UNAVHardware::~UNAVHardware() {
    serial_->clearCallback(HASHMAP_MOTION);
    serial_->clearCallback(HASHMAP_MOTOR);
    clearVectorPacketRequest();
    clearParameterPacketRequest();
    clearTimerEvent();
}

void UNAVHardware::addParameter(std::vector<information_packet_t>* list_send) {
    //Constraint/Left
    ROS_INFO("%s/constraint/%s",joint_string.c_str(),left_string.c_str());
    if (nh_.hasParam(joint_string + "/constraint" + left_string)) {
        ROS_INFO("Sync parameter constraint/%s: ROS -> ROBOT",left_string.c_str());
        motor_t constraint = get_constraint(left_string);
        list_send->push_back(serial_->createDataPacket(CONSTRAINT_L, HASHMAP_MOTOR, (abstract_message_u*) & constraint));
    } else {
        ROS_INFO("Sync parameter constraint/%s: ROBOT -> ROS",left_string.c_str());
        list_send->push_back(serial_->createPacket(CONSTRAINT_L, REQUEST, HASHMAP_MOTOR));
    }
    //Constraint/Right
    ROS_INFO("%s/constraint/%s",joint_string.c_str(),right_string.c_str());
    if (nh_.hasParam(joint_string + "/constraint" + right_string)) {
        ROS_INFO("Sync parameter constraint/%s: ROS -> ROBOT",right_string.c_str());
        motor_t constraint = get_constraint(right_string);
        list_send->push_back(serial_->createDataPacket(CONSTRAINT_R, HASHMAP_MOTOR, (abstract_message_u*) & constraint));
    } else {
        ROS_INFO("Sync parameter constraint/%s: ROBOT -> ROS",right_string.c_str());
        list_send->push_back(serial_->createPacket(CONSTRAINT_R, REQUEST, HASHMAP_MOTOR));
    }
    //Load PID/Left
    if (nh_.hasParam("pid/" + left_string)) {
        ROS_INFO("Sync parameter pid/%s: ROS -> ROBOT", left_string.c_str());
        pid_control_t pid_l = get_pid(left_string);
        list_send->push_back(serial_->createDataPacket(PID_CONTROL_L, HASHMAP_MOTOR, (abstract_message_u*) & pid_l));
    } else {
        ROS_INFO("Sync parameter pid/%s: ROBOT -> ROS", left_string.c_str());
        list_send->push_back(serial_->createPacket(PID_CONTROL_L, REQUEST, HASHMAP_MOTOR));
    }
    //Load PID/Right
    if (nh_.hasParam("pid/" + right_string)) {
        ROS_INFO("Sync parameter pid/%s: ROS -> ROBOT", right_string.c_str());
        pid_control_t pid_r = get_pid(right_string);
        list_send->push_back(serial_->createDataPacket(PID_CONTROL_R, HASHMAP_MOTOR, (abstract_message_u*) & pid_r));
    } else {
        ROS_INFO("Sync parameter pid/%s: ROBOT -> ROS", right_string.c_str());
        list_send->push_back(serial_->createPacket(PID_CONTROL_R, REQUEST, HASHMAP_MOTOR));
    }
    //Parameter unicycle
    if (nh_.hasParam("structure")) {
        ROS_INFO("Sync parameter structure: ROS -> ROBOT");
        parameter_unicycle_t parameter_unicycle = get_unicycle_parameter();
        list_send->push_back(serial_->createDataPacket(PARAMETER_UNICYCLE, HASHMAP_MOTION, (abstract_message_u*) & parameter_unicycle));
    } else {
        ROS_INFO("Sync parameter structure: ROBOT -> ROS");
        list_send->push_back(serial_->createPacket(PARAMETER_UNICYCLE, REQUEST, HASHMAP_MOTION));
    }
    //Parameter motors LEFT
    if (nh_.hasParam(joint_string + "/" + left_string)) {
        ROS_INFO("Sync parameter parameter motor/%s: ROS -> ROBOT",left_string.c_str());
        parameter_motor_t parameter_motor = get_motor_parameter(left_string);
        list_send->push_back(serial_->createDataPacket(PARAMETER_MOTOR_L, HASHMAP_MOTOR, (abstract_message_u*) & parameter_motor));
    } else {
        ROS_INFO("Sync parameter parameter motor/%s: ROBOT -> ROS",left_string.c_str());
        list_send->push_back(serial_->createPacket(PARAMETER_MOTOR_L, REQUEST, HASHMAP_MOTOR));
    }
    //Parameter motors RIGHT
    if (nh_.hasParam(joint_string + "/" + right_string)) {
        ROS_INFO("Sync parameter parameter motor/%s: ROS -> ROBOT",right_string.c_str());
        parameter_motor_t parameter_motor = get_motor_parameter(right_string);
        list_send->push_back(serial_->createDataPacket(PARAMETER_MOTOR_R, HASHMAP_MOTOR, (abstract_message_u*) & parameter_motor));
    } else {
        ROS_INFO("Sync parameter parameter motor/%s: ROBOT -> ROS",right_string.c_str());
        list_send->push_back(serial_->createPacket(PARAMETER_MOTOR_R, REQUEST, HASHMAP_MOTOR));
    }
    //Names ele
    if (!nh_.hasParam(joint_string + "/back_emf")) {
        ROS_INFO("Sync parameter %s/back_emf: set", joint_string.c_str());
        nh_.setParam(joint_string + "/back_emf/" + left_string, 1.0);
        nh_.setParam(joint_string + "/back_emf/" + right_string, 1.0);
    }
    //Set emergency/Left
    if (nh_.hasParam(emergency_string)) {
        ROS_INFO("Sync parameter %s/%s: ROS -> ROBOT", emergency_string.c_str(),left_string.c_str());
        emergency_t emergency = get_emergency(left_string);
        list_send->push_back(serial_->createDataPacket(EMERGENCY_L, HASHMAP_MOTOR, (abstract_message_u*) & emergency));
    } else {
        ROS_INFO("Sync parameter %s/%s: ROBOT -> ROS", emergency_string.c_str(),left_string.c_str());
        list_send->push_back(serial_->createPacket(EMERGENCY_L, REQUEST, HASHMAP_MOTOR));
    }
    //Set emergency/Right
    if (nh_.hasParam(emergency_string)) {
        ROS_INFO("Sync parameter %s/%s: ROS -> ROBOT", emergency_string.c_str(),right_string.c_str());
        emergency_t emergency = get_emergency(right_string);
        list_send->push_back(serial_->createDataPacket(EMERGENCY_R, HASHMAP_MOTOR, (abstract_message_u*) & emergency));
    } else {
        ROS_INFO("Sync parameter %s/%s: ROBOT -> ROS", emergency_string.c_str(), right_string.c_str());
        list_send->push_back(serial_->createPacket(EMERGENCY_R, REQUEST, HASHMAP_MOTOR));
    }
}

void UNAVHardware::motorPacket(const unsigned char& command, const abstract_message_u* packet) {
    switch (command) {
    case CONSTRAINT_L:
        nh_.setParam(joint_string + "/constraint/" + left_string + "/velocity", packet->motor.velocity);
        break;
    case CONSTRAINT_R:
        nh_.setParam(joint_string + "/constraint/" + right_string + "/velocity", packet->motor.velocity);
        break;
    case PARAMETER_MOTOR_L:
        nh_.setParam(joint_string + "/" + left_string + "/cpr", packet->parameter_motor.cpr);
        nh_.setParam(joint_string + "/" + left_string + "/ratio", packet->parameter_motor.ratio);
        nh_.setParam(joint_string + "/" + left_string + "/encoder_pos", packet->parameter_motor.encoder_pos);
        nh_.setParam(joint_string + "/" + left_string + "/volt_bridge", packet->parameter_motor.volt_bridge);
        nh_.setParam(joint_string + "/" + left_string + "/encoder_swap", packet->parameter_motor.versus);
        nh_.setParam(joint_string + "/" + left_string + "/default_enable", packet->parameter_motor.enable_set);
        break;
    case PARAMETER_MOTOR_R:
        nh_.setParam(joint_string + "/" + right_string + "/cpr", packet->parameter_motor.cpr);
        nh_.setParam(joint_string + "/" + right_string + "/ratio", packet->parameter_motor.ratio);
        nh_.setParam(joint_string + "/" + right_string + "/encoder_pos", packet->parameter_motor.encoder_pos);
        nh_.setParam(joint_string + "/" + right_string + "/volt_bridge", packet->parameter_motor.volt_bridge);
        nh_.setParam(joint_string + "/" + right_string + "/encoder_swap", packet->parameter_motor.versus);
        nh_.setParam(joint_string + "/" + right_string + "/default_enable", packet->parameter_motor.enable_set);
        break;
    case EMERGENCY_L:
        nh_.setParam(emergency_string + "/" + left_string + "/bridge_off", packet->emergency.bridge_off);
        nh_.setParam(emergency_string + "/" + left_string + "/slope_time", packet->emergency.slope_time);
        nh_.setParam(emergency_string + "/" + left_string + "/timeout", ((double) packet->emergency.timeout) / 1000.0);
        break;
    case EMERGENCY_R:
        nh_.setParam(emergency_string + "/" + right_string + "/bridge_off", packet->emergency.bridge_off);
        nh_.setParam(emergency_string + "/" + right_string + "/slope_time", packet->emergency.slope_time);
        nh_.setParam(emergency_string + "/" + right_string + "/timeout", ((double) packet->emergency.timeout) / 1000.0);
        break;
    case PID_CONTROL_L:
        name_pid = "pid/" + left_string + "/";
        nh_.setParam(name_pid + "P", packet->pid.kp);
        nh_.setParam(name_pid + "I", packet->pid.ki);
        nh_.setParam(name_pid + "D", packet->pid.kd);
        break;
    case PID_CONTROL_R:
        name_pid = "pid/" + right_string + "/";
        nh_.setParam(name_pid + "P", packet->pid.kp);
        nh_.setParam(name_pid + "I", packet->pid.ki);
        nh_.setParam(name_pid + "D", packet->pid.kd);
        break;
    case VEL_MOTOR_MIS_L:
        joints_[LEFT].velocity = ((double) packet->motor_control) / 1000;
        break;
    case VEL_MOTOR_MIS_R:
        joints_[RIGHT].velocity = ((double) packet->motor_control) / 1000;
        break;
    case MOTOR_L:
        joints_[LEFT].effort = packet->motor.torque;
        joints_[LEFT].position = packet->motor.position;
        joints_[LEFT].velocity = ((double) packet->motor.velocity) / 1000;
        break;
    case MOTOR_R:
        joints_[RIGHT].effort = packet->motor.torque;
        joints_[RIGHT].position = packet->motor.position;
        joints_[RIGHT].velocity = ((double) packet->motor.velocity) / 1000;
        break;
    }
}

void UNAVHardware::motionPacket(const unsigned char& command, const abstract_message_u* packet) {
    switch (command) {
    case PARAMETER_UNICYCLE:
        nh_.setParam("structure/" + wheelbase_string, packet->parameter_unicycle.wheelbase);
        nh_.setParam("structure/" + radius_string + "/" + right_string, packet->parameter_unicycle.radius_r);
        nh_.setParam("structure/" + radius_string + "/" + left_string, packet->parameter_unicycle.radius_l);
        nh_.setParam("odo_mis_step", packet->parameter_unicycle.sp_min);
        break;
    case COORDINATE:
    case VELOCITY:
    case VELOCITY_MIS:
    case ENABLE:
    default:
        break;
    }
}

void UNAVHardware::registerControlInterfaces() {
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

void UNAVHardware::updateJointsFromHardware() {
    //Send a list of request about position and velocities
    std::vector<information_packet_t> list_send;
    list_send.push_back(serial_->createPacket(MOTOR_L, REQUEST, HASHMAP_MOTOR));
    list_send.push_back(serial_->createPacket(MOTOR_R, REQUEST, HASHMAP_MOTOR));
    state_controller_t status = STATE_CONTROL_VELOCITY;
    motor_control_t velocity = 1000;
    list_send.push_back(serial_->createDataPacket(ENABLE_MOTOR_L, HASHMAP_MOTOR, (abstract_message_u*) & status));

    list_send.push_back(serial_->createDataPacket(VEL_MOTOR_L, HASHMAP_MOTOR, (abstract_message_u*) & velocity));
    try {
        serial_->parserSendPacket(list_send, 3, boost::posix_time::millisec(200));
    } catch (exception &e) {
        ROS_ERROR("%s", e.what());
    }
    //TODO Add a function to collect all commands
    ROS_INFO("Update Joints");
}

void UNAVHardware::writeCommandsToHardware() {
    ROS_INFO("Write to Hardware");


}

void UNAVHardware::timerEvent(const ros::TimerEvent& event) {
//    // Send Odometry message
//    if (pub_odom.getNumSubscribers() >= 1) {
//        sendOdometry(&meas_velocity, &pose);
//    }
//    // Send JointState message
//    if (pub_joint.getNumSubscribers() >= 1) {
//        sendJointState(&motor_left, &motor_right);
//    }
}

bool UNAVHardware::aliveOperation(const ros::TimerEvent& event, std::vector<information_packet_t>* list_send) {
//    if (sub_twist.getNumPublishers() >= 1) {
//        list_send->push_back(serial_->createDataPacket(VEL_MOTOR_L, HASHMAP_MOTION, (abstract_message_u*) & velocity_ref[0]));
//        list_send->push_back(serial_->createDataPacket(VEL_MOTOR_R, HASHMAP_MOTION, (abstract_message_u*) & velocity_ref[1]));
//        return true;
//    } else {
//        if (status[0] != STATE_CONTROL_EMERGENCY || status[1] != STATE_CONTROL_EMERGENCY) {
//            status[0] = STATE_CONTROL_EMERGENCY;
//            status[1] = STATE_CONTROL_EMERGENCY;
//            list_send->push_back(serial_->createDataPacket(ENABLE_MOTOR_L, HASHMAP_MOTION, (abstract_message_u*) & status[0]));
//            list_send->push_back(serial_->createDataPacket(ENABLE_MOTOR_R, HASHMAP_MOTION, (abstract_message_u*) & status[1]));
//            return true;
//        } else {
//            return false;
//        }
//    }
}

void UNAVHardware::updatePacket(std::vector<information_packet_t>* list_send) {
    std::string packet_string;
//    if ((pub_pose.getNumSubscribers() >= 1) || (pub_odom.getNumSubscribers() >= 1)) {
//        packet_string += "Odo ";
//        list_send->push_back(serial_->createPacket(COORDINATE, REQUEST, HASHMAP_MOTION));
//    }
//    if (pub_twist.getNumSubscribers() >= 1) {
//        packet_string += "Vel ";
//        list_send->push_back(serial_->createPacket(VELOCITY, REQUEST, HASHMAP_MOTION));
//    }
//    if (pub_enable.getNumSubscribers() >= 1) {
//        packet_string += "Ena ";
//        list_send->push_back(serial_->createPacket(ENABLE, REQUEST, HASHMAP_MOTION));
//    }
//    if (pub_odom.getNumSubscribers() >= 1) {
//        packet_string += "VelMis ";
//        list_send->push_back(serial_->createPacket(VELOCITY_MIS, REQUEST, HASHMAP_MOTION));
//    }
//    if ((pub_motor_left.getNumSubscribers() >= 1) || (pub_joint.getNumSubscribers() >= 1)) {
//        packet_string += "MotL ";
//        list_send->push_back(serial_->createPacket(MOTOR_L, REQUEST, HASHMAP_MOTION));
//    }
//    if ((pub_motor_right.getNumSubscribers() >= 1) || (pub_joint.getNumSubscribers() >= 1)) {
//        packet_string += "MotR ";
//        list_send->push_back(serial_->createPacket(MOTOR_R, REQUEST, HASHMAP_MOTION));
//    }
    //    ROS_INFO("[ %s]", packet_string.c_str());
}

pid_control_t UNAVHardware::get_pid(std::string name) {
    pid_control_t pid;
    double temp;
    std::string name_pid = "pid/" + name + "/";
    nh_.getParam(name_pid + "P", temp);
    pid.kp = temp;
    nh_.getParam(name_pid + "I", temp);
    pid.ki = temp;
    nh_.getParam(name_pid + "D", temp);
    pid.kd = temp;
    return pid;
}

parameter_motor_t UNAVHardware::get_motor_parameter(std::string name) {
    parameter_motor_t parameter;
    double temp;
    int temp2;

    nh_.getParam(joint_string + "/" + name + "/cpr", temp);
    parameter.cpr = temp;
    nh_.getParam(joint_string + "/" + name + "/ratio", temp);
    parameter.ratio = temp;
    nh_.getParam(joint_string + "/" + name + "/encoder_pos", temp);
    parameter.encoder_pos = temp;
    nh_.getParam(joint_string + "/" + name + "/volt_bridge", temp);
    parameter.volt_bridge = temp;
    nh_.getParam(joint_string + "/" + name + "/versus", temp2);
    parameter.versus = temp2;
    nh_.getParam(joint_string + "/" + name + "/default_enable", temp2);
    parameter.enable_set = temp2;
    return parameter;
}

parameter_unicycle_t UNAVHardware::get_unicycle_parameter() {
    parameter_unicycle_t parameter;
    double temp;

    nh_.getParam("structure/" + wheelbase_string, temp);
    parameter.wheelbase = temp;
    nh_.getParam("structure/" + radius_string + "/" + right_string, temp);
    parameter.radius_r = temp;
    nh_.getParam("structure/" + radius_string + "/" + left_string, temp);
    parameter.radius_l = temp;
    nh_.getParam("odo_mis_step", temp);
    parameter.sp_min = temp;
    return parameter;
}

emergency_t UNAVHardware::get_emergency(std::string name) {
    emergency_t emergency;
    double temp;
    nh_.getParam(emergency_string + name + "/bridge_off", temp);
    emergency.bridge_off = temp;
    nh_.getParam(emergency_string + name + "/slope_time", temp);
    emergency.slope_time = temp;
    nh_.getParam(emergency_string + name + "/timeout", temp);
    emergency.timeout = (int16_t) (temp * 1000);
    return emergency;
}

motor_t UNAVHardware::get_constraint(std::string name) {
    motor_t constraint;
    int temp;

    nh_.getParam(joint_string + "/constraint/" + name + "/velocity", temp);
    constraint.velocity = (int16_t) temp;
    return constraint;
}

bool UNAVHardware::pidServiceCallback(ros_serial_bridge::Update::Request &req, ros_serial_bridge::Update::Response&) {
    std::string name = req.name;
    pid_control_t pid;
    std::vector<information_packet_t> list_send;
    ROS_INFO("PID UPDATE");
    if ((name.compare(left_string) == 0) || (name.compare(all_string) == 0)) {
        pid = get_pid(left_string);
        list_send.push_back(serial_->createDataPacket(PID_CONTROL_L, HASHMAP_MOTOR, (abstract_message_u*) & pid));
    }
    if ((name.compare(right_string) == 0) || (name.compare(all_string) == 0)) {
        pid = get_pid(right_string);
        list_send.push_back(serial_->createDataPacket(PID_CONTROL_R, HASHMAP_MOTOR, (abstract_message_u*) & pid));
    }
    try {
        serial_->parserSendPacket(list_send, 3, boost::posix_time::millisec(200));
    } catch (exception &e) {
        ROS_ERROR("%s", e.what());
    }
    return true;
}

bool UNAVHardware::parameterServiceCallback(ros_serial_bridge::Update::Request &req, ros_serial_bridge::Update::Response&) {
    std::string name = req.name;
    parameter_motor_t parameter_motor;
    std::vector<information_packet_t> list_send;
    ROS_INFO("PARAMETER UPDATE");
    if ((name.compare(left_string) == 0) || (name.compare(all_string) == 0)) {
        parameter_motor = get_motor_parameter(left_string);
        list_send.push_back(serial_->createDataPacket(PARAMETER_MOTOR_L, HASHMAP_MOTOR, (abstract_message_u*) & parameter_motor));
    }
    if ((name.compare(right_string) == 0) || (name.compare(all_string) == 0)) {
        parameter_motor = get_motor_parameter(right_string);
        list_send.push_back(serial_->createDataPacket(PARAMETER_MOTOR_R, HASHMAP_MOTOR, (abstract_message_u*) & parameter_motor));
    }
    if ((name.compare(paramenter_unicycle_string) == 0) || (name.compare(all_string) == 0)) {
        parameter_unicycle_t parameter_unicycle = get_unicycle_parameter();
        list_send.push_back(serial_->createDataPacket(PARAMETER_UNICYCLE, HASHMAP_MOTION, (abstract_message_u*) & parameter_unicycle));
    }
    try {
        serial_->parserSendPacket(list_send, 3, boost::posix_time::millisec(200));
    } catch (exception &e) {
        ROS_ERROR("%s", e.what());
    }
    return true;
}

bool UNAVHardware::constraintServiceCallback(ros_serial_bridge::Update::Request &req, ros_serial_bridge::Update::Response&) {
    //motor_t constraint = get_constraint();
    std::string name = req.name;
    motor_t constraint;
    std::vector<information_packet_t> list_send;
    ROS_INFO("CONSTRAINT UPDATE");
    if ((name.compare(left_string) == 0) || (name.compare(all_string) == 0)) {
        constraint = get_constraint(left_string);
        list_send.push_back(serial_->createDataPacket(CONSTRAINT_L, HASHMAP_MOTOR, (abstract_message_u*) & constraint));
    }
    if ((name.compare(right_string) == 0) || (name.compare(all_string) == 0)) {
        constraint = get_constraint(right_string);
        list_send.push_back(serial_->createDataPacket(CONSTRAINT_R, HASHMAP_MOTOR, (abstract_message_u*) & constraint));
    }
    try {
        serial_->parserSendPacket(list_send, 3, boost::posix_time::millisec(200));
    } catch (exception &e) {
        ROS_ERROR("%s", e.what());
    }
    return true;
}

bool UNAVHardware::emergencyServiceCallback(ros_serial_bridge::Update::Request &req, ros_serial_bridge::Update::Response&) {
    std::string name = req.name;
    emergency_t emergency;
    std::vector<information_packet_t> list_send;
    if ((name.compare(left_string) == 0) || (name.compare(all_string) == 0)) {
        emergency = get_emergency(left_string);
        list_send.push_back(serial_->createDataPacket(EMERGENCY_L, HASHMAP_MOTOR, (abstract_message_u*) & emergency));
    }
    if ((name.compare(right_string) == 0) || (name.compare(all_string) == 0)) {
        emergency = get_emergency(right_string);
        list_send.push_back(serial_->createDataPacket(EMERGENCY_R, HASHMAP_MOTOR, (abstract_message_u*) & emergency));
    }
    try {
        serial_->parserSendPacket(list_send, 3, boost::posix_time::millisec(200));
    } catch (exception &e) {
        ROS_ERROR("%s", e.what());
    }
    return true;
}
