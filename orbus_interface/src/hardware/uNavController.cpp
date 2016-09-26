#include "hardware/uNavController.h"

uNavController::uNavController(orbus::serial_controller *serial) : GenericController(serial)
{
    /// Added all callback to receive information about messages
    bool initCallback = mSerial->addCallback(&uNavController::motorFrame, this, HASHMAP_MOTOR);

}

void uNavController::updateJointsFromHardware()
{

}

void uNavController::writeCommandsToHardware(ros::Duration period)
{

}


void uNavController::motorFrame(unsigned char option, unsigned char type, unsigned char command, message_abstract_u message) {
    ROS_INFO_STREAM("uNav Controller I'm here");
}

