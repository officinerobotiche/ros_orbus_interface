#include "hardware/GenericController.h"

GenericController::GenericController(orbus::serial_controller *serial) : mSerial(serial)
{
    serial->addCallback(&GenericController::systemFrame, this, HASHMAP_MOTOR);


}

packet_information_t GenericController::systemFrame(unsigned char option, unsigned char type, unsigned char command, message_abstract_u message) {
    ROS_INFO_STREAM("I'm here");
}
