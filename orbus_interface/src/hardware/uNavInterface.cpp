
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
    Motor *motor0 = new Motor(private_mNh, serial, 0);
    Motor *motor1 = new Motor(private_mNh, serial, 1);
    // Add Two motors on list
    list_motor.push_back(motor0);
    list_motor.push_back(motor1);

    //TODO send a message to the board to update the name
    name_board = "uNav";
}

bool uNavInterface::updateDiagnostics()
{
    if(serial_status)
    {
        ROS_DEBUG_STREAM("Update diagnostic");
        // Force update all diagnostic parts
        diagnostic_updater.force_update();
    }
    else
    {
        ROS_ERROR("Error connection! Try to connect again ...");
        // Send list of Command
        serial_status = mSerial->sendList();
        if(serial_status)
            ROS_INFO("... connected!");
    }
    return serial_status;
}

void uNavInterface::initializeMotors()
{
    for(unsigned i=0; i < list_motor.size(); ++i)
    {
        // Get motor
        Motor *motor_obj = list_motor.at(i);
        // Initialize all components
        motor_obj->initializeMotor();
        ROS_DEBUG_STREAM("Motor [" << (int) i << "] Initialized");
    }
    // Send list of Command
    serial_status = mSerial->sendList();
}

void uNavInterface::initializeInterfaces()
{

    ROS_INFO_STREAM("Name board: " << name_board);
    diagnostic_updater.setHardwareID(name_board);

    for(unsigned i=0; i < list_motor.size(); ++i)
    {
        // Get motor
        Motor *motor_obj = list_motor.at(i);
        // Register interface
        motor_obj->registerControlInterfaces(&joint_state_interface, &velocity_joint_interface, urdf);
        //Add motor in diagnostic updater
        diagnostic_updater.add(*motor_obj);
        ROS_DEBUG_STREAM("Motor [" << (int) i << "] Registered");
    }
    ROS_INFO_STREAM("Send all Constraint configuration");
    // Send list of Command
    serial_status = mSerial->sendList();

    /// Register interfaces
    registerInterface(&joint_state_interface);
    registerInterface(&velocity_joint_interface);
}

bool uNavInterface::updateJointsFromHardware()
{
    ROS_DEBUG_STREAM("Get measure from uNav");
    for(unsigned i=0; i < list_motor.size(); ++i)
    {
        // Get motor
        Motor *motor_obj = list_motor.at(i);
        motor_obj->addRequestMeasure();
    }
    //Send all messages
    serial_status = mSerial->sendList();
    return serial_status;
}

bool uNavInterface::writeCommandsToHardware(ros::Duration period)
{
    ROS_DEBUG_STREAM("Write command to uNav");
    for(unsigned i=0; i < list_motor.size(); ++i)
    {
        // Get motor
        Motor *motor_obj = list_motor.at(i);
        motor_obj->writeCommandsToHardware(period);
    }
    //Send all messages
    serial_status = mSerial->sendList();
    return serial_status;
}

void uNavInterface::allMotorsFrame(unsigned char option, unsigned char type, unsigned char command, message_abstract_u message)
{

    motor_command_map_t motor;
    motor.command_message = command;
    int number_motor = (int) motor.bitset.motor;
    ROS_DEBUG_STREAM("Frame [Option: " << option << ", HashMap: " << type << ", Nmotor: " << number_motor << ", Command: " << (int) motor.bitset.command << "]");
    if(number_motor < list_motor.size())
    {
        // Get motor
        Motor *motor_obj = list_motor.at(number_motor);
        // Update information
        motor_obj->motorFrame(option, type, motor.bitset.command, message.motor);
    }
    else
    {
        ROS_WARN_STREAM("Error enything motor is initialized for Motor: " << number_motor);
    }

}

}
