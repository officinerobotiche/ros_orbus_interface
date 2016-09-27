
#include <string>

#include <urdf/model.h>
#include <urdf_parser/urdf_parser.h>

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>


#include "hardware/uNavInterface.h"

namespace ORInterface
{

uNavInterface::uNavInterface(const ros::NodeHandle &nh, const ros::NodeHandle &private_nh, orbus::serial_controller *serial)
    : GenericInterface(nh, private_nh, serial)
{
    /// Added all callback to receive information about messages
    bool initCallback = mSerial->addCallback(&uNavInterface::allMotorsFrame, this, HASHMAP_MOTOR);

    /// Load URDF from robot_description
    if(nh.hasParam("/robot_description"))
    {
        ROS_INFO_STREAM("/robot_description found!");
        std::string urdf_string;
        nh.getParam("/robot_description", urdf_string);
        urdf = urdf::parseURDF(urdf_string);
    } else
    {
        ROS_WARN_STREAM("/robot_description NOT found!");
    }

    //Initialize motors
    Motor motor0(private_mNh, serial, 0);
    Motor motor1(private_mNh, serial, 1);
    // Add Two motors on list
    list_motor.push_back(motor0);
    list_motor.push_back(motor1);
}

void uNavInterface::initialize()
{
    //Initialize motors
    for(unsigned i=0; i < list_motor.size(); ++i)
    {
        // Get motor
        Motor motor_obj = list_motor.at(i);
        motor_obj.initializeMotor();
    }

    /// Register all control interface avaiable
    registerControlInterfaces();
}

void uNavInterface::registerControlInterfaces()
{
    ROS_INFO_STREAM("Register Control interfaces");
    for(unsigned i=0; i < list_motor.size(); ++i)
    {
        // Get motor
        Motor motor_obj = list_motor.at(i);
        // Register interface
        motor_obj.registerControlInterfaces(joint_state_interface, velocity_joint_interface, urdf);
    }
    ROS_INFO_STREAM("Send all configuration");
    // Send list of Command
    mSerial->sendList();

    /// Register interfaces
    registerInterface(&joint_state_interface);
    registerInterface(&velocity_joint_interface);
}

void uNavInterface::updateJointsFromHardware()
{
    ROS_DEBUG_STREAM("Update joint information from uNav");
    for(unsigned i=0; i < list_motor.size(); ++i)
    {
        // Get motor
        Motor motor_obj = list_motor.at(i);
        motor_obj.addRequestMeasure();
    }
    mSerial->sendList();
}

void uNavInterface::writeCommandsToHardware(ros::Duration period)
{
    ROS_DEBUG_STREAM("Write command to uNav");
    for(unsigned i=0; i < list_motor.size(); ++i)
    {
        // Get motor
        Motor motor_obj = list_motor.at(i);
        motor_obj.writeCommandsToHardware(period);
    }
    mSerial->sendList();
}

void uNavInterface::allMotorsFrame(unsigned char option, unsigned char type, unsigned char command, message_abstract_u message)
{

    motor_command_map_t motor;
    motor.command_message = command;
    int number_motor = (int) motor.bitset.motor;
    ROS_INFO_STREAM("Frame [Option: " << option << ", HashMap: " << type << ", Nmotor: " << number_motor << ", Command: " << (int) motor.bitset.command << "]");
    if(number_motor < list_motor.size())
    {
        // Get motor
        Motor motor_obj = list_motor.at(number_motor);
        // Update information
        motor_obj.motorFrame(option, type, motor.bitset.command, message.motor);
    }
    else
    {
        ROS_WARN_STREAM("Error enything motor is initialized for Motor: " << number_motor);
    }

}

}
