#include "configurator/GenericConfigurator.h"

using namespace std;

GenericConfigurator::GenericConfigurator(const ros::NodeHandle &nh, orbus::serial_controller *serial, unsigned int number)
    : nh_(nh)
    , mSerial(serial)
{
    // Set command message
    mCommand.bitset.motor = number;
    // Set false in the first run
    setup_ = false;
}

void GenericConfigurator::SendParameterToBoard(message_abstract_u message)
{
    packet_information_t frame = CREATE_PACKET_DATA(mCommand.command_message, HASHMAP_MOTOR, message);
    // Add packet in the frame and send
    if(mSerial->addFrame(frame)->sendList())
    {
        ROS_INFO_STREAM("Write PARAM:" << mName << " in uNav");
    }
    else
    {
        ROS_ERROR_STREAM("Unable to receive packet from uNav");
    }
}
