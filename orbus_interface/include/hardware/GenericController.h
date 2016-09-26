#ifndef GENERICCONTROLLER_H
#define GENERICCONTROLLER_H

#include <ros/ros.h>
#include "hardware/serial_controller.h"

class GenericController
{
public:
    GenericController(orbus::serial_controller *serial);

private:
    packet_information_t systemFrame(unsigned char option, unsigned char type, unsigned char command, message_abstract_u message);

private:
    orbus::serial_controller *mSerial;
};

#endif // GENERICCONTROLLER_H
