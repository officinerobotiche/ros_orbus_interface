/* 
 * File:   ROSMotionController.cpp
 * Author: Raffaello Bonghi
 * 
 * Created on 15 November 2013, 18:34
 */

#include "hardware/UNAVHardware.h"
#include <limits>

#include <boost/assign/list_of.hpp>
// Boost header needed:
#include <boost/lexical_cast.hpp>

#define NUMBER_PUB 10
#define SGN(x)  ( ((x) < 0) ?  -1 : ( ((x) == 0 ) ? 0 : 1) )

using namespace std;

namespace
{
  const uint8_t LEFT = 0, RIGHT = 1;
}

UNAVHardware::UNAVHardware(const ros::NodeHandle& nh, const ros::NodeHandle &private_nh, ParserPacket* serial)
: ORBHardware(nh, private_nh, serial) {

    // Verify correct type board
    if (type_board_.compare("Motor Control") != 0) {
        throw (controller_exception("Other board: " + type_board_));
    }

    // Added all callback to receive information about messages
    serial->addCallback(&UNAVHardware::motionPacket, this, HASHMAP_MOTION);
    serial->addCallback(&UNAVHardware::motorPacket, this, HASHMAP_MOTOR);
    addParameterPacketRequest(&UNAVHardware::addParameter, this);

    // Load all parameters
    loadParameter();

    // Register all control interface avaiable
    registerControlInterfaces();
}

UNAVHardware::~UNAVHardware() {
    serial_->clearCallback(HASHMAP_MOTION);
    serial_->clearCallback(HASHMAP_MOTOR);
    clearParameterPacketRequest();
}

void UNAVHardware::addParameter(std::vector<packet_information_t>* list_send) {
    motor_command_map_t command;
    std::string number_motor_string;
    for(unsigned int i=0; i < NUM_MOTORS; ++i) {
        command.bitset.motor = i;
        number_motor_string = "motor_" + boost::lexical_cast<std::string>(i);
        /// PID
        joints_[i].configurator_pid = new MotorPIDConfigurator(private_nh_, number_motor_string, i, serial_);
        /// Parameter motor
        joints_[i].configurator_param = new MotorParamConfigurator(private_nh_, number_motor_string, i, serial_);

    }
//        list_send->push_back(serial_->createDataPacket(command, HASHMAP_MOTOR, (abstract_message_u*) & constraint));
//        list_send->push_back(serial_->createDataPacket(EMERGENCY_L, HASHMAP_MOTOR, (abstract_message_u*) & emergency));
//        list_send->push_back(serial_->createDataPacket(PARAMETER_UNICYCLE, HASHMAP_MOTION, (abstract_message_u*) & parameter_unicycle));
}

void UNAVHardware::motorPacket(const unsigned char& command, const message_abstract_u* packet) {
    motor_command_.command_message = command;
    int number_motor = (int) motor_command_.bitset.motor;
    std::string number_motor_string = boost::lexical_cast<std::string>(number_motor);
    switch (motor_command_.bitset.command) {
    case MOTOR_CONSTRAINT:
        //nh_.setParam(joint_string + "/constraint/" + left_string + "/velocity", packet->motor.velocity);
        break;
        break;
    case MOTOR_EMERGENCY:
//        nh_.setParam(emergency_string + "/" + left_string + "/bridge_off", packet->emergency.bridge_off);
//        nh_.setParam(emergency_string + "/" + left_string + "/slope_time", packet->emergency.slope_time);
//        nh_.setParam(emergency_string + "/" + left_string + "/timeout", ((double) packet->emergency.timeout) / 1000.0);
        break;
        break;
    case MOTOR_VEL_MEAS:
        joints_[number_motor].velocity = ((double) packet->motor_control) / 1000;
        break;
    case MOTOR:
        joints_[number_motor].effort = packet->motor.torque;
        joints_[number_motor].position = packet->motor.position;
        joints_[number_motor].velocity = ((double) packet->motor.velocity) / 1000;
        //ROS_INFO_STREAM("[" << (int)motor_command_.bitset.motor << "] Position: " <<  joints_[motor_command_.bitset.motor].position << " - Velocity: " << joints_[motor_command_.bitset.motor].velocity);
        break;
    }
}

void UNAVHardware::motionPacket(const unsigned char& command, const message_abstract_u* packet) {
    switch (command) {
    case PARAMETER_UNICYCLE:
//        nh_.setParam("structure/" + wheelbase_string, packet->parameter_unicycle.wheelbase);
//        nh_.setParam("structure/" + radius_string + "/" + right_string, packet->parameter_unicycle.radius_r);
//        nh_.setParam("structure/" + radius_string + "/" + left_string, packet->parameter_unicycle.radius_l);
//        nh_.setParam("odo_mis_step", packet->parameter_unicycle.sp_min);
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
    ros::V_string joint_names = boost::assign::list_of("Left")("Right");
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
    //ROS_INFO("Update Joints");
    //Send a list of request about position and velocities
    std::vector<packet_information_t> list_send;
    motor_command_map_t motor_command;
    motor_command.bitset.command = MOTOR;
    for(int i = 0; i < NUM_MOTORS; ++i) {
        motor_command.bitset.motor = i;
        list_send.push_back(serial_->createPacket(motor_command.command_message, PACKET_REQUEST, HASHMAP_MOTOR));
    }
    try {
        serial_->parserSendPacket(list_send, 3, boost::posix_time::millisec(200));
    } catch (exception &e) {
        ROS_ERROR("%s", e.what());
    }
    //TODO Add a function to collect all commands

}

void UNAVHardware::writeCommandsToHardware() {
    //ROS_INFO("Write to Hardware");
    motor_state_t status = STATE_CONTROL_VELOCITY;
    motor_control_t velocity = 1000;
    std::vector<packet_information_t> list_send;
    motor_command_map_t motor_command;
    unsigned char command;
    for(int i = 0; i < NUM_MOTORS; ++i) {
        motor_command.bitset.motor = i;
//        motor_command.bitset.command = MOTOR_STATE;
//        command = motor_command.command_message;
//        ROS_INFO_STREAM("Number: " << (int)command);
//        list_send.push_back(serial_->createDataPacket(command, HASHMAP_MOTOR, (message_abstract_u*) & status));
        motor_command.bitset.command = MOTOR_VEL_REF;
        command = motor_command.command_message;
        //ROS_INFO_STREAM("Number: " << (int)command);
        list_send.push_back(serial_->createDataPacket(command, HASHMAP_MOTOR, (message_abstract_u*) & velocity));
    }
    try {
        serial_->parserSendPacket(list_send, 3, boost::posix_time::millisec(200));
    } catch (exception &e) {
        ROS_ERROR("%s", e.what());
    }
}

//parameter_unicycle_t UNAVHardware::get_unicycle_parameter() {
//    parameter_unicycle_t parameter;
//    double temp;

//    nh_.getParam("structure/" + wheelbase_string, temp);
//    parameter.wheelbase = temp;
//    nh_.getParam("structure/" + radius_string + "/" + right_string, temp);
//    parameter.radius_r = temp;
//    nh_.getParam("structure/" + radius_string + "/" + left_string, temp);
//    parameter.radius_l = temp;
//    nh_.getParam("odo_mis_step", temp);
//    parameter.sp_min = temp;
//    return parameter;
//}

motor_emergency_t UNAVHardware::get_emergency(std::string name) {
    motor_emergency_t emergency;
//    double temp;
//    nh_.getParam(emergency_string + name + "/bridge_off", temp);
//    emergency.bridge_off = temp;
//    nh_.getParam(emergency_string + name + "/slope_time", temp);
//    emergency.slope_time = temp;
//    nh_.getParam(emergency_string + name + "/timeout", temp);
//    emergency.timeout = (int16_t) (temp * 1000);
    return emergency;
}

motor_t UNAVHardware::get_constraint(std::string name) {
    motor_t constraint;
//    int temp;

//    nh_.getParam(joint_string + "/constraint/" + name + "/velocity", temp);
//    constraint.velocity = (int16_t) temp;
    return constraint;
}

//bool UNAVHardware::pidServiceCallback(ros_serial_bridge::Update::Request &req, ros_serial_bridge::Update::Response&) {
//    return true;
//}

//bool UNAVHardware::parameterServiceCallback(ros_serial_bridge::Update::Request &req, ros_serial_bridge::Update::Response&) {
//    return true;
//}

//bool UNAVHardware::constraintServiceCallback(ros_serial_bridge::Update::Request &req, ros_serial_bridge::Update::Response&) {
//    return true;
//}

//bool UNAVHardware::emergencyServiceCallback(ros_serial_bridge::Update::Request &req, ros_serial_bridge::Update::Response&) {
//    return true;
//}
