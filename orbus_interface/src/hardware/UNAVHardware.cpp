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

#include "urdf_parser/urdf_parser.h"

#define NUMBER_PUB 10
#define SGN(x)  ( ((x) < 0) ?  -1 : ( ((x) == 0 ) ? 0 : 1) )

using namespace std;

namespace
{
  const uint8_t LEFT = 0, RIGHT = 1;
}

UNAVHardware::UNAVHardware(const ros::NodeHandle& nh, const ros::NodeHandle &private_nh, ParserPacket* serial)
: ORBHardware(nh, private_nh, serial) {

    /// Verify correct type board
    if (type_board_.compare("Motor Control") != 0) {
        throw (controller_exception("Other board: " + type_board_));
    }

    /// Added all callback to receive information about messages
    serial->addCallback(&UNAVHardware::motorPacket, this, HASHMAP_MOTOR);
    addParameterPacketRequest(&UNAVHardware::addParameter, this);

    /// Load all parameters
    loadParameter();

    /// Register all control interface avaiable
    registerControlInterfaces();
}

UNAVHardware::~UNAVHardware() {
    serial_->clearCallback(HASHMAP_MOTION);
    serial_->clearCallback(HASHMAP_MOTOR);
    clearParameterPacketRequest();
}

void UNAVHardware::registerControlInterfaces() {
    /// Build ad array with name of joints
//    std::string joints[2];
//    nh_.param<std::string>("joint/left", joints[0], "Left");
//    nh_.param<std::string>("joint/right", joints[1], "Right");
//    ros::V_string joint_names = boost::assign::list_of(joints[0])(joints[1]);
    ros::V_string joint_names = boost::assign::list_of("Left")("Right");
    /// Build harware interfaces
    for (unsigned int i = 0; i < joint_names.size(); i++)
    {
        /// Joint hardware interface
        hardware_interface::JointStateHandle joint_state_handle(joint_names[i],
                                                                &joints_[i].position, &joints_[i].velocity, &joints_[i].effort);
        joint_state_interface_.registerHandle(joint_state_handle);

        /// Differential drive interface
        hardware_interface::JointHandle joint_handle(
                    joint_state_handle, &joints_[i].velocity_command);
        velocity_joint_interface_.registerHandle(joint_handle);

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
        if(urdf_ != NULL) {
            boost::shared_ptr<const urdf::Joint> urdf_joint = urdf_->getJoint(joint_names[i]);
            const bool urdf_limits_ok = getJointLimits(urdf_joint, limits);
            const bool urdf_soft_limits_ok = getSoftJointLimits(urdf_joint, soft_limits);
            if(urdf_limits_ok) {
                ROS_INFO_STREAM("LOAD " << joint_names[i] << " limits from URDF");
            }
            if(urdf_soft_limits_ok) {
                ROS_INFO_STREAM("LOAD " << joint_names[i] << " soft limits from URDF");
            }
        }

        // Populate (soft) joint limits from the ros parameter server
        // Limits specified in the parameter server overwrite existing values in 'limits' and 'soft_limits'
        // Limits not specified in the parameter server preserve their existing values
        const bool rosparam_limits_ok = getJointLimits(joint_names[i], nh_, limits);
        if(rosparam_limits_ok) {
            ROS_INFO_STREAM("LOAD " << joint_names[i] << " limits from ROSPARAM");
        }

        joint_limits_interface::VelocityJointSoftLimitsHandle handle(joint_handle, // We read the state and read/write the command
                                                                     limits,       // Limits spec
                                                                     soft_limits);  // Soft limits spec

        vel_limits_interface_.registerHandle(handle);

    }
    /// Register interfaces
    registerInterface(&joint_state_interface_);
    registerInterface(&velocity_joint_interface_);

}

void UNAVHardware::updateJointsFromHardware() {
    //ROS_INFO("Update Joints");
    /// Send a list of request about position and velocities
    list_send_.clear();     ///< Clear list of commands
    motor_command_.bitset.command = MOTOR; ///< Set message to receive measure information
    for(int i = 0; i < NUM_MOTORS; ++i) {
        motor_command_.bitset.motor = i;
        list_send_.push_back(serial_->createPacket(motor_command_.command_message, PACKET_REQUEST, HASHMAP_MOTOR));
    }
    try {
        serial_->parserSendPacket(list_send_, 3, boost::posix_time::millisec(200));
    } catch (exception &e) {
        ROS_ERROR("%s", e.what());
    }
}

void UNAVHardware::writeCommandsToHardware(ros::Duration period) {
    //ROS_INFO("Write to Hardware");

    // Enforce joint limits for all registered handles
    // Note: one can also enforce limits on a per-handle basis: handle.enforceLimits(period)
    vel_limits_interface_.enforceLimits(period);

    list_send_.clear();     ///< Clear list of commands
    motor_command_.bitset.command = MOTOR_VEL_REF; ///< Set command to velocity control
    for(int i = 0; i < NUM_MOTORS; ++i) {
        //Build a command message
        motor_command_.bitset.motor = i;
        /// Convert radiant velocity in milliradiant
        long int velocity_long = (long int) joints_[i].velocity_command*1000;
        motor_control_t velocity;
        // >>>>> Saturation on 16 bit values
        if(velocity_long > 32767) {
            velocity = 32767;
        } else if (velocity_long < -32768) {
            velocity_long = -32768;
        } else {
            velocity = (motor_control_t) velocity_long;
        }
        // <<<<< Saturation on 16 bit values
        list_send_.push_back(serial_->createDataPacket(motor_command_.command_message, HASHMAP_MOTOR, (message_abstract_u*) & velocity));
    }
    /// Send message
    try {
        serial_->parserSendPacket(list_send_, 3, boost::posix_time::millisec(200));
    } catch (exception &e) {
        ROS_ERROR("%s", e.what());
    }
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
        /// Emergency motor
        joints_[i].configurator_emergency = new MotorEmergencyConfigurator(private_nh_, number_motor_string, i, serial_);
    }

    /// Load URDF from robot_description
    if(nh_.hasParam("/robot_description")) {
        std::string urdf_string;
        nh_.getParam("/robot_description", urdf_string);
        urdf_ = urdf::parseURDF(urdf_string);
    }
}

void UNAVHardware::motorPacket(const unsigned char& command, const message_abstract_u* packet) {
    motor_command_.command_message = command;
    switch (motor_command_.bitset.command) {
    case MOTOR:
        joints_[motor_command_.bitset.motor].effort = packet->motor.torque;
        joints_[motor_command_.bitset.motor].position += packet->motor.position_delta;
        joints_[motor_command_.bitset.motor].velocity = ((double) packet->motor.velocity) / 1000;
        break;
    }
}
