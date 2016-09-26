
#include <string>


#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>


#include "hardware/uNavController.h"

namespace ORController
{

uNavController::uNavController(const ros::NodeHandle &nh, orbus::serial_controller *serial) : GenericController(nh, serial)
{
    /// Added all callback to receive information about messages
    bool initCallback = mSerial->addCallback(&uNavController::allMotorsFrame, this, HASHMAP_MOTOR);

    // Add Two motor
    list_motor.push_back(new Motor(nh, serial, 0));
    list_motor.push_back(new Motor(nh, serial, 1));

}

void uNavController::registerControlInterfaces()
{
    ROS_INFO_STREAM("Register Control interfaces");
    for(unsigned i=0; i < list_motor.size(); ++i)
    {
        // Get motor
        Motor* motor_obj = list_motor.at(i);
        // Register interface
        motor_obj->registerControlInterfaces(joint_state_interface, velocity_joint_interface, urdf);
    }

    mSerial->start();

    /// Register interfaces
    registerInterface(&joint_state_interface);
    registerInterface(&velocity_joint_interface);
}

void uNavController::updateJointsFromHardware()
{

}

void uNavController::writeCommandsToHardware(ros::Duration period)
{

}

void uNavController::allMotorsFrame(unsigned char option, unsigned char type, unsigned char command, message_abstract_u message)
{
    ROS_INFO_STREAM("uNav Controller I'm here");
    motor_command_map_t motor;
    motor.command_message = command;
    int number_motor = (int) motor.bitset.motor;
    if(number_motor < list_motor.size())
    {
        // Get motor
        Motor* motor_obj = list_motor.at(number_motor);
        // Update information
        motor_obj->motorFrame(option, type, motor.bitset.command, message);
    }

}

}
